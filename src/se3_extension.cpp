#define DUCKDB_EXTENSION_MAIN

#include "se3_extension.hpp"

#include "duckdb.hpp"
#include "duckdb/function/scalar_function.hpp"

#include <cmath>

namespace duckdb {

// ------------------------- Types -------------------------
static LogicalType Vec3Type() {
	vector<pair<string, LogicalType>> f;
	f.emplace_back("x", LogicalType::DOUBLE);
	f.emplace_back("y", LogicalType::DOUBLE);
	f.emplace_back("z", LogicalType::DOUBLE);
	return LogicalType::STRUCT(f);
}

static LogicalType QuatType() {
	vector<pair<string, LogicalType>> f;
	f.emplace_back("w", LogicalType::DOUBLE);
	f.emplace_back("x", LogicalType::DOUBLE);
	f.emplace_back("y", LogicalType::DOUBLE);
	f.emplace_back("z", LogicalType::DOUBLE);
	return LogicalType::STRUCT(f);
}

static LogicalType WType() {
	vector<pair<string, LogicalType>> f;
	f.emplace_back("t", Vec3Type());
	f.emplace_back("q", QuatType());
	return LogicalType::STRUCT(f);
}

// ------------------------- Validity helpers -------------------------
static inline bool RowIsValid(const UnifiedVectorFormat &uf, idx_t i) {
	return uf.validity.RowIsValid(uf.sel->get_index(i));
}

// ------------------------- Quaternion kernels -------------------------
static inline void QMul(double aw, double ax, double ay, double az, double bw, double bx, double by, double bz,
                        double &rw, double &rx, double &ry, double &rz) {
	rw = aw * bw - ax * bx - ay * by - az * bz;
	rx = aw * bx + ax * bw + ay * bz - az * by;
	ry = aw * by - ax * bz + ay * bw + az * bx;
	rz = aw * bz + ax * by - ay * bx + az * bw;
}

static inline void QRotate(double qw, double qx, double qy, double qz, double vx, double vy, double vz, double &ox,
                           double &oy, double &oz) {
	// t = 2*cross(qv, v)
	const double tx1 = 2.0 * (qy * vz - qz * vy);
	const double ty1 = 2.0 * (qz * vx - qx * vz);
	const double tz1 = 2.0 * (qx * vy - qy * vx);
	// cross(qv, t)
	const double cx = qy * tz1 - qz * ty1;
	const double cy = qz * tx1 - qx * tz1;
	const double cz = qx * ty1 - qy * tx1;
	// v' = v + qw*t + cross(qv,t)
	ox = vx + qw * tx1 + cx;
	oy = vy + qw * ty1 + cy;
	oz = vz + qw * tz1 + cz;
}

static inline void QInvRotate(double qw, double qx, double qy, double qz, double vx, double vy, double vz, double &ox,
                              double &oy, double &oz) {
	QRotate(qw, -qx, -qy, -qz, vx, vy, vz, ox, oy, oz);
}

// ------------------------- Output writers -------------------------
static inline void PrepareStructChildrenFlat(Vector &struct_vec) {
	struct_vec.SetVectorType(VectorType::FLAT_VECTOR);
	auto &ent = StructVector::GetEntries(struct_vec);
	for (auto &e : ent) {
		e->SetVectorType(VectorType::FLAT_VECTOR);
	}
}

// vec3 result: STRUCT(x,y,z)
static inline void PrepareVec3Out(Vector &result, double *&ox, double *&oy, double *&oz) {
	PrepareStructChildrenFlat(result);
	auto &ent = StructVector::GetEntries(result);
	ox = FlatVector::GetData<double>(*ent[0]);
	oy = FlatVector::GetData<double>(*ent[1]);
	oz = FlatVector::GetData<double>(*ent[2]);
}

// quat result: STRUCT(w,x,y,z)
static inline void PrepareQuatOut(Vector &result, double *&ow, double *&ox, double *&oy, double *&oz) {
	PrepareStructChildrenFlat(result);
	auto &ent = StructVector::GetEntries(result);
	ow = FlatVector::GetData<double>(*ent[0]);
	ox = FlatVector::GetData<double>(*ent[1]);
	oy = FlatVector::GetData<double>(*ent[2]);
	oz = FlatVector::GetData<double>(*ent[3]);
}

// W result: STRUCT(t vec3, q quat)
static inline void PrepareWOut(Vector &result, double *&tx, double *&ty, double *&tz, double *&qw, double *&qx,
                               double *&qy, double *&qz) {
	PrepareStructChildrenFlat(result);
	auto &went = StructVector::GetEntries(result);

	// t
	auto &tent = StructVector::GetEntries(*went[0]);
	for (auto &e : tent) {
		e->SetVectorType(VectorType::FLAT_VECTOR);
	}
	tx = FlatVector::GetData<double>(*tent[0]);
	ty = FlatVector::GetData<double>(*tent[1]);
	tz = FlatVector::GetData<double>(*tent[2]);

	// q
	auto &qent = StructVector::GetEntries(*went[1]);
	for (auto &e : qent) {
		e->SetVectorType(VectorType::FLAT_VECTOR);
	}
	qw = FlatVector::GetData<double>(*qent[0]);
	qx = FlatVector::GetData<double>(*qent[1]);
	qy = FlatVector::GetData<double>(*qent[2]);
	qz = FlatVector::GetData<double>(*qent[3]);
}

// ------------------------- Functions -------------------------

// quat_from_axis_angle(axis:vec3, th:DOUBLE) -> quat
static void QuatFromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &axis_v = input.data[0];
	auto &axis_ent = StructVector::GetEntries(axis_v);
	auto &th_v = input.data[1];

	UnifiedVectorFormat axis_uf, ax_uf, ay_uf, az_uf, th_uf;
	axis_v.ToUnifiedFormat(n, axis_uf);
	axis_ent[0]->ToUnifiedFormat(n, ax_uf);
	axis_ent[1]->ToUnifiedFormat(n, ay_uf);
	axis_ent[2]->ToUnifiedFormat(n, az_uf);
	th_v.ToUnifiedFormat(n, th_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ax_sel = ax_uf.sel;
	auto *ay = (const double *)ay_uf.data;
	auto *ay_sel = ay_uf.sel;
	auto *az = (const double *)az_uf.data;
	auto *az_sel = az_uf.sel;
	auto *th = (const double *)th_uf.data;
	auto *th_sel = th_uf.sel;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, ow, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = axis_uf.validity.AllValid() && ax_uf.validity.AllValid() && ay_uf.validity.AllValid() &&
	                       az_uf.validity.AllValid() && th_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(axis_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) ||
			    !RowIsValid(th_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double thv = th[th_sel->get_index(i)];
		const double h = 0.5 * thv;
		const double c = std::cos(h);
		const double s = std::sin(h);

		ow[i] = c;
		ox[i] = ax[ax_sel->get_index(i)] * s;
		oy[i] = ay[ay_sel->get_index(i)] * s;
		oz[i] = az[az_sel->get_index(i)] * s;
	}
}

// qmul(qA,qB) -> quat
static void QMulFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf;
	UnifiedVectorFormat bw_uf, bx_uf, by_uf, bz_uf;
	UnifiedVectorFormat a_uf, b_uf;

	a_v.ToUnifiedFormat(n, a_uf);
	b_v.ToUnifiedFormat(n, b_uf);
	a[0]->ToUnifiedFormat(n, aw_uf);
	a[1]->ToUnifiedFormat(n, ax_uf);
	a[2]->ToUnifiedFormat(n, ay_uf);
	a[3]->ToUnifiedFormat(n, az_uf);

	b[0]->ToUnifiedFormat(n, bw_uf);
	b[1]->ToUnifiedFormat(n, bx_uf);
	b[2]->ToUnifiedFormat(n, by_uf);
	b[3]->ToUnifiedFormat(n, bz_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *aw_sel = aw_uf.sel;
	auto *ax = (const double *)ax_uf.data;
	auto *ax_sel = ax_uf.sel;
	auto *ay = (const double *)ay_uf.data;
	auto *ay_sel = ay_uf.sel;
	auto *az = (const double *)az_uf.data;
	auto *az_sel = az_uf.sel;

	auto *bw = (const double *)bw_uf.data;
	auto *bw_sel = bw_uf.sel;
	auto *bx = (const double *)bx_uf.data;
	auto *bx_sel = bx_uf.sel;
	auto *by = (const double *)by_uf.data;
	auto *by_sel = by_uf.sel;
	auto *bz = (const double *)bz_uf.data;
	auto *bz_sel = bz_uf.sel;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, ow, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = a_uf.validity.AllValid() && b_uf.validity.AllValid() && aw_uf.validity.AllValid() &&
	                       ax_uf.validity.AllValid() && ay_uf.validity.AllValid() && az_uf.validity.AllValid() &&
	                       bw_uf.validity.AllValid() && bx_uf.validity.AllValid() && by_uf.validity.AllValid() &&
	                       bz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) ||
			    !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) || !RowIsValid(bw_uf, i) || !RowIsValid(bx_uf, i) ||
			    !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		double rw, rx, ry, rz;
		QMul(aw[aw_sel->get_index(i)], ax[ax_sel->get_index(i)], ay[ay_sel->get_index(i)], az[az_sel->get_index(i)],
		     bw[bw_sel->get_index(i)], bx[bx_sel->get_index(i)], by[by_sel->get_index(i)], bz[bz_sel->get_index(i)], rw,
		     rx, ry, rz);
		ow[i] = rw;
		ox[i] = rx;
		oy[i] = ry;
		oz[i] = rz;
	}
}

// qconj(q) -> quat
static void QConjFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	auto &q_v = input.data[0];
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat w_uf, x_uf, y_uf, z_uf;
	UnifiedVectorFormat q_uf;
	q_v.ToUnifiedFormat(n, q_uf);
	q[0]->ToUnifiedFormat(n, w_uf);
	q[1]->ToUnifiedFormat(n, x_uf);
	q[2]->ToUnifiedFormat(n, y_uf);
	q[3]->ToUnifiedFormat(n, z_uf);

	auto *w = (const double *)w_uf.data;
	auto *w_sel = w_uf.sel;
	auto *x = (const double *)x_uf.data;
	auto *x_sel = x_uf.sel;
	auto *y = (const double *)y_uf.data;
	auto *y_sel = y_uf.sel;
	auto *z = (const double *)z_uf.data;
	auto *z_sel = z_uf.sel;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, ow, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && w_uf.validity.AllValid() && x_uf.validity.AllValid() &&
	                       y_uf.validity.AllValid() && z_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(x_uf, i) || !RowIsValid(y_uf, i) ||
			    !RowIsValid(z_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		ow[i] = w[w_sel->get_index(i)];
		ox[i] = -x[x_sel->get_index(i)];
		oy[i] = -y[y_sel->get_index(i)];
		oz[i] = -z[z_sel->get_index(i)];
	}
}

// qnorm2(q) -> DOUBLE
static void QNorm2Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	auto &q_v = input.data[0];
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat w_uf, x_uf, y_uf, z_uf;
	UnifiedVectorFormat q_uf;
	q_v.ToUnifiedFormat(n, q_uf);
	q[0]->ToUnifiedFormat(n, w_uf);
	q[1]->ToUnifiedFormat(n, x_uf);
	q[2]->ToUnifiedFormat(n, y_uf);
	q[3]->ToUnifiedFormat(n, z_uf);

	auto *w = (const double *)w_uf.data;
	auto *w_sel = w_uf.sel;
	auto *x = (const double *)x_uf.data;
	auto *x_sel = x_uf.sel;
	auto *y = (const double *)y_uf.data;
	auto *y_sel = y_uf.sel;
	auto *z = (const double *)z_uf.data;
	auto *z_sel = z_uf.sel;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	auto *out = FlatVector::GetData<double>(result);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && w_uf.validity.AllValid() && x_uf.validity.AllValid() &&
	                       y_uf.validity.AllValid() && z_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(x_uf, i) || !RowIsValid(y_uf, i) ||
			    !RowIsValid(z_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double ww = w[w_sel->get_index(i)];
		const double xx = x[x_sel->get_index(i)];
		const double yy = y[y_sel->get_index(i)];
		const double zz = z[z_sel->get_index(i)];
		out[i] = ww * ww + xx * xx + yy * yy + zz * zz;
	}
}

// se3_identity() -> W
static void Se3IdentityFn(DataChunk &input, ExpressionState &, Vector &result) {
	(void)input;
	const idx_t n = input.size();

	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, tx, ty, tz, qw, qx, qy, qz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	for (idx_t i = 0; i < n; i++) {
		tx[i] = 0.0;
		ty[i] = 0.0;
		tz[i] = 0.0;
		qw[i] = 1.0;
		qx[i] = 0.0;
		qy[i] = 0.0;
		qz[i] = 0.0;
	}
}

// se3_make(t,q) -> W
static void Se3MakeFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	auto &t_v = input.data[0];
	auto &q_v = input.data[1];
	auto &t = StructVector::GetEntries(t_v);
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat t_uf, q_uf;

	t_v.ToUnifiedFormat(n, t_uf);
	q_v.ToUnifiedFormat(n, q_uf);
	t[0]->ToUnifiedFormat(n, tx_uf);
	t[1]->ToUnifiedFormat(n, ty_uf);
	t[2]->ToUnifiedFormat(n, tz_uf);

	q[0]->ToUnifiedFormat(n, qw_uf);
	q[1]->ToUnifiedFormat(n, qx_uf);
	q[2]->ToUnifiedFormat(n, qy_uf);
	q[3]->ToUnifiedFormat(n, qz_uf);

	auto *itx = (const double *)tx_uf.data;
	auto *itx_sel = tx_uf.sel;
	auto *ity = (const double *)ty_uf.data;
	auto *ity_sel = ty_uf.sel;
	auto *itz = (const double *)tz_uf.data;
	auto *itz_sel = tz_uf.sel;

	auto *iqw = (const double *)qw_uf.data;
	auto *iqw_sel = qw_uf.sel;
	auto *iqx = (const double *)qx_uf.data;
	auto *iqx_sel = qx_uf.sel;
	auto *iqy = (const double *)qy_uf.data;
	auto *iqy_sel = qy_uf.sel;
	auto *iqz = (const double *)qz_uf.data;
	auto *iqz_sel = qz_uf.sel;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = t_uf.validity.AllValid() && q_uf.validity.AllValid() && tx_uf.validity.AllValid() &&
	                       ty_uf.validity.AllValid() && tz_uf.validity.AllValid() && qw_uf.validity.AllValid() &&
	                       qx_uf.validity.AllValid() && qy_uf.validity.AllValid() && qz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) ||
			    !RowIsValid(qz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		otx[i] = itx[itx_sel->get_index(i)];
		oty[i] = ity[ity_sel->get_index(i)];
		otz[i] = itz[itz_sel->get_index(i)];
		oqw[i] = iqw[iqw_sel->get_index(i)];
		oqx[i] = iqx[iqx_sel->get_index(i)];
		oqy[i] = iqy[iqy_sel->get_index(i)];
		oqz[i] = iqz[iqz_sel->get_index(i)];
	}
}

// se3_from_axis_angle(t, axis, th) -> W
static void Se3FromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &axis_v = input.data[1];
	auto &t = StructVector::GetEntries(t_v);
	auto &axis = StructVector::GetEntries(axis_v);
	auto &th_v = input.data[2];

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat ax_uf, ay_uf, az_uf, th_uf;
	UnifiedVectorFormat t_uf, axis_uf;

	t_v.ToUnifiedFormat(n, t_uf);
	axis_v.ToUnifiedFormat(n, axis_uf);
	t[0]->ToUnifiedFormat(n, tx_uf);
	t[1]->ToUnifiedFormat(n, ty_uf);
	t[2]->ToUnifiedFormat(n, tz_uf);

	axis[0]->ToUnifiedFormat(n, ax_uf);
	axis[1]->ToUnifiedFormat(n, ay_uf);
	axis[2]->ToUnifiedFormat(n, az_uf);
	th_v.ToUnifiedFormat(n, th_uf);

	auto *itx = (const double *)tx_uf.data;
	auto *itx_sel = tx_uf.sel;
	auto *ity = (const double *)ty_uf.data;
	auto *ity_sel = ty_uf.sel;
	auto *itz = (const double *)tz_uf.data;
	auto *itz_sel = tz_uf.sel;

	auto *ax = (const double *)ax_uf.data;
	auto *ax_sel = ax_uf.sel;
	auto *ay = (const double *)ay_uf.data;
	auto *ay_sel = ay_uf.sel;
	auto *az = (const double *)az_uf.data;
	auto *az_sel = az_uf.sel;
	auto *th = (const double *)th_uf.data;
	auto *th_sel = th_uf.sel;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = t_uf.validity.AllValid() && axis_uf.validity.AllValid() && tx_uf.validity.AllValid() &&
	                       ty_uf.validity.AllValid() && tz_uf.validity.AllValid() && ax_uf.validity.AllValid() &&
	                       ay_uf.validity.AllValid() && az_uf.validity.AllValid() && th_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t_uf, i) || !RowIsValid(axis_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) ||
			    !RowIsValid(th_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double thv = th[th_sel->get_index(i)];
		const double h = 0.5 * thv;
		const double c = std::cos(h);
		const double s = std::sin(h);

		otx[i] = itx[itx_sel->get_index(i)];
		oty[i] = ity[ity_sel->get_index(i)];
		otz[i] = itz[itz_sel->get_index(i)];

		oqw[i] = c;
		oqx[i] = ax[ax_sel->get_index(i)] * s;
		oqy[i] = ay[ay_sel->get_index(i)] * s;
		oqz[i] = az[az_sel->get_index(i)] * s;
	}
}

// se3_apply(W, p) -> vec3   (W(p) = R_q(p + t))
static void Se3ApplyFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &w_v = input.data[0];
	auto &went = StructVector::GetEntries(w_v);      // W: {t,q}
	auto &tent = StructVector::GetEntries(*went[0]); // t
	auto &qent = StructVector::GetEntries(*went[1]); // q
	auto &p_v = input.data[1];
	auto &pent = StructVector::GetEntries(p_v); // p

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat px_uf, py_uf, pz_uf;
	UnifiedVectorFormat w_uf, t_uf, q_uf, p_uf;

	w_v.ToUnifiedFormat(n, w_uf);
	went[0]->ToUnifiedFormat(n, t_uf);
	went[1]->ToUnifiedFormat(n, q_uf);
	p_v.ToUnifiedFormat(n, p_uf);
	tent[0]->ToUnifiedFormat(n, tx_uf);
	tent[1]->ToUnifiedFormat(n, ty_uf);
	tent[2]->ToUnifiedFormat(n, tz_uf);

	qent[0]->ToUnifiedFormat(n, qw_uf);
	qent[1]->ToUnifiedFormat(n, qx_uf);
	qent[2]->ToUnifiedFormat(n, qy_uf);
	qent[3]->ToUnifiedFormat(n, qz_uf);

	pent[0]->ToUnifiedFormat(n, px_uf);
	pent[1]->ToUnifiedFormat(n, py_uf);
	pent[2]->ToUnifiedFormat(n, pz_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *tx_sel = tx_uf.sel;
	auto *ty = (const double *)ty_uf.data;
	auto *ty_sel = ty_uf.sel;
	auto *tz = (const double *)tz_uf.data;
	auto *tz_sel = tz_uf.sel;

	auto *qw = (const double *)qw_uf.data;
	auto *qw_sel = qw_uf.sel;
	auto *qx = (const double *)qx_uf.data;
	auto *qx_sel = qx_uf.sel;
	auto *qy = (const double *)qy_uf.data;
	auto *qy_sel = qy_uf.sel;
	auto *qz = (const double *)qz_uf.data;
	auto *qz_sel = qz_uf.sel;

	auto *px = (const double *)px_uf.data;
	auto *px_sel = px_uf.sel;
	auto *py = (const double *)py_uf.data;
	auto *py_sel = py_uf.sel;
	auto *pz = (const double *)pz_uf.data;
	auto *pz_sel = pz_uf.sel;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = w_uf.validity.AllValid() && t_uf.validity.AllValid() && q_uf.validity.AllValid() &&
	                       p_uf.validity.AllValid() && tx_uf.validity.AllValid() && ty_uf.validity.AllValid() &&
	                       tz_uf.validity.AllValid() && qw_uf.validity.AllValid() && qx_uf.validity.AllValid() &&
	                       qy_uf.validity.AllValid() && qz_uf.validity.AllValid() && px_uf.validity.AllValid() &&
	                       py_uf.validity.AllValid() && pz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(p_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) ||
			    !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(px_uf, i) ||
			    !RowIsValid(py_uf, i) || !RowIsValid(pz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double tx_i = tx[tx_sel->get_index(i)];
		const double ty_i = ty[ty_sel->get_index(i)];
		const double tz_i = tz[tz_sel->get_index(i)];

		const double qw_i = qw[qw_sel->get_index(i)];
		const double qx_i = qx[qx_sel->get_index(i)];
		const double qy_i = qy[qy_sel->get_index(i)];
		const double qz_i = qz[qz_sel->get_index(i)];

		const double ux = px[px_sel->get_index(i)] + tx_i;
		const double uy = py[py_sel->get_index(i)] + ty_i;
		const double uz = pz[pz_sel->get_index(i)] + tz_i;

		QRotate(qw_i, qx_i, qy_i, qz_i, ux, uy, uz, ox[i], oy[i], oz[i]);
	}
}

// se3_apply(t, p) -> vec3   (p + t)
static void Se3ApplyTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &p_v = input.data[1];
	auto &t = StructVector::GetEntries(t_v);
	auto &p = StructVector::GetEntries(p_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, px_uf, py_uf, pz_uf;
	UnifiedVectorFormat t_uf, p_uf;
	t_v.ToUnifiedFormat(n, t_uf);
	p_v.ToUnifiedFormat(n, p_uf);

	t[0]->ToUnifiedFormat(n, tx_uf);
	t[1]->ToUnifiedFormat(n, ty_uf);
	t[2]->ToUnifiedFormat(n, tz_uf);
	p[0]->ToUnifiedFormat(n, px_uf);
	p[1]->ToUnifiedFormat(n, py_uf);
	p[2]->ToUnifiedFormat(n, pz_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;
	auto *px = (const double *)px_uf.data;
	auto *py = (const double *)py_uf.data;
	auto *pz = (const double *)pz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = t_uf.validity.AllValid() && p_uf.validity.AllValid() && tx_uf.validity.AllValid() &&
	                       ty_uf.validity.AllValid() && tz_uf.validity.AllValid() && px_uf.validity.AllValid() &&
	                       py_uf.validity.AllValid() && pz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t_uf, i) || !RowIsValid(p_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i) || !RowIsValid(px_uf, i) || !RowIsValid(py_uf, i) || !RowIsValid(pz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		ox[i] = px[px_uf.sel->get_index(i)] + tx[tx_uf.sel->get_index(i)];
		oy[i] = py[py_uf.sel->get_index(i)] + ty[ty_uf.sel->get_index(i)];
		oz[i] = pz[pz_uf.sel->get_index(i)] + tz[tz_uf.sel->get_index(i)];
	}
}

// se3_apply(q, p) -> vec3   (R_q(p))
static void Se3ApplyQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &q_v = input.data[0];
	auto &p_v = input.data[1];
	auto &q = StructVector::GetEntries(q_v);
	auto &p = StructVector::GetEntries(p_v);

	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf, px_uf, py_uf, pz_uf;
	UnifiedVectorFormat q_uf, p_uf;
	q_v.ToUnifiedFormat(n, q_uf);
	p_v.ToUnifiedFormat(n, p_uf);

	q[0]->ToUnifiedFormat(n, qw_uf);
	q[1]->ToUnifiedFormat(n, qx_uf);
	q[2]->ToUnifiedFormat(n, qy_uf);
	q[3]->ToUnifiedFormat(n, qz_uf);
	p[0]->ToUnifiedFormat(n, px_uf);
	p[1]->ToUnifiedFormat(n, py_uf);
	p[2]->ToUnifiedFormat(n, pz_uf);

	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *px = (const double *)px_uf.data;
	auto *py = (const double *)py_uf.data;
	auto *pz = (const double *)pz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && p_uf.validity.AllValid() && qw_uf.validity.AllValid() &&
	                       qx_uf.validity.AllValid() && qy_uf.validity.AllValid() && qz_uf.validity.AllValid() &&
	                       px_uf.validity.AllValid() && py_uf.validity.AllValid() && pz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(p_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(px_uf, i) || !RowIsValid(py_uf, i) ||
			    !RowIsValid(pz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);
		const idx_t ipx_i = px_uf.sel->get_index(i);
		const idx_t ipy_i = py_uf.sel->get_index(i);
		const idx_t ipz_i = pz_uf.sel->get_index(i);

		QRotate(qw[iqw_i], qx[iqx_i], qy[iqy_i], qz[iqz_i], px[ipx_i], py[ipy_i], pz[ipz_i], ox[i], oy[i], oz[i]);
	}
}

// se3_inv(t) -> t (negation)
static void Se3InvTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &t = StructVector::GetEntries(t_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, t_uf;
	t_v.ToUnifiedFormat(n, t_uf);
	t[0]->ToUnifiedFormat(n, tx_uf);
	t[1]->ToUnifiedFormat(n, ty_uf);
	t[2]->ToUnifiedFormat(n, tz_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid =
	    t_uf.validity.AllValid() && tx_uf.validity.AllValid() && ty_uf.validity.AllValid() && tz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		ox[i] = -tx[tx_uf.sel->get_index(i)];
		oy[i] = -ty[ty_uf.sel->get_index(i)];
		oz[i] = -tz[tz_uf.sel->get_index(i)];
	}
}

// se3_inv(q) -> q (conjugate)
static void Se3InvQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &q_v = input.data[0];
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat w_uf, x_uf, y_uf, z_uf, q_uf;
	q_v.ToUnifiedFormat(n, q_uf);
	q[0]->ToUnifiedFormat(n, w_uf);
	q[1]->ToUnifiedFormat(n, x_uf);
	q[2]->ToUnifiedFormat(n, y_uf);
	q[3]->ToUnifiedFormat(n, z_uf);

	auto *w = (const double *)w_uf.data;
	auto *x = (const double *)x_uf.data;
	auto *y = (const double *)y_uf.data;
	auto *z = (const double *)z_uf.data;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, ow, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && w_uf.validity.AllValid() && x_uf.validity.AllValid() &&
	                       y_uf.validity.AllValid() && z_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(x_uf, i) || !RowIsValid(y_uf, i) ||
			    !RowIsValid(z_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		ow[i] = w[w_uf.sel->get_index(i)];
		ox[i] = -x[x_uf.sel->get_index(i)];
		oy[i] = -y[y_uf.sel->get_index(i)];
		oz[i] = -z[z_uf.sel->get_index(i)];
	}
}

// se3_inv(W) -> W such that W_inv(W(p)) = p
static void Se3InvWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &w_v = input.data[0];
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(*went[0]);
	auto &qent = StructVector::GetEntries(*went[1]);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat w_uf, t_uf, q_uf;

	w_v.ToUnifiedFormat(n, w_uf);
	went[0]->ToUnifiedFormat(n, t_uf);
	went[1]->ToUnifiedFormat(n, q_uf);

	tent[0]->ToUnifiedFormat(n, tx_uf);
	tent[1]->ToUnifiedFormat(n, ty_uf);
	tent[2]->ToUnifiedFormat(n, tz_uf);
	qent[0]->ToUnifiedFormat(n, qw_uf);
	qent[1]->ToUnifiedFormat(n, qx_uf);
	qent[2]->ToUnifiedFormat(n, qy_uf);
	qent[3]->ToUnifiedFormat(n, qz_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;
	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = w_uf.validity.AllValid() && t_uf.validity.AllValid() && q_uf.validity.AllValid() &&
	                       tx_uf.validity.AllValid() && ty_uf.validity.AllValid() && tz_uf.validity.AllValid() &&
	                       qw_uf.validity.AllValid() && qx_uf.validity.AllValid() && qy_uf.validity.AllValid() &&
	                       qz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(tx_uf, i) ||
			    !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);

		double rtx, rty, rtz;
		QRotate(qw[iqw_i], qx[iqx_i], qy[iqy_i], qz[iqz_i], tx[itx_i], ty[ity_i], tz[itz_i], rtx, rty, rtz);

		otx[i] = -rtx;
		oty[i] = -rty;
		otz[i] = -rtz;

		oqw[i] = qw[iqw_i];
		oqx[i] = -qx[iqx_i];
		oqy[i] = -qy[iqy_i];
		oqz[i] = -qz[iqz_i];
	}
}

// se3_compose overloads: result equals applying B, then A
// W = A ∘ B  with convention W(p)=R_q(p+t)

// se3_compose(WA, WB) -> W
static void Se3ComposeWWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a_ent = StructVector::GetEntries(a_v);
	auto &b_ent = StructVector::GetEntries(b_v);
	auto &at = StructVector::GetEntries(*a_ent[0]);
	auto &aq = StructVector::GetEntries(*a_ent[1]);
	auto &bt = StructVector::GetEntries(*b_ent[0]);
	auto &bq = StructVector::GetEntries(*b_ent[1]);

	UnifiedVectorFormat atx_uf, aty_uf, atz_uf, aqw_uf, aqx_uf, aqy_uf, aqz_uf;
	UnifiedVectorFormat btx_uf, bty_uf, btz_uf, bqw_uf, bqx_uf, bqy_uf, bqz_uf;
	UnifiedVectorFormat a_uf, b_uf, at_uf, aq_uf, bt_uf, bq_uf;

	a_v.ToUnifiedFormat(n, a_uf);
	b_v.ToUnifiedFormat(n, b_uf);
	a_ent[0]->ToUnifiedFormat(n, at_uf);
	a_ent[1]->ToUnifiedFormat(n, aq_uf);
	b_ent[0]->ToUnifiedFormat(n, bt_uf);
	b_ent[1]->ToUnifiedFormat(n, bq_uf);

	at[0]->ToUnifiedFormat(n, atx_uf);
	at[1]->ToUnifiedFormat(n, aty_uf);
	at[2]->ToUnifiedFormat(n, atz_uf);
	aq[0]->ToUnifiedFormat(n, aqw_uf);
	aq[1]->ToUnifiedFormat(n, aqx_uf);
	aq[2]->ToUnifiedFormat(n, aqy_uf);
	aq[3]->ToUnifiedFormat(n, aqz_uf);

	bt[0]->ToUnifiedFormat(n, btx_uf);
	bt[1]->ToUnifiedFormat(n, bty_uf);
	bt[2]->ToUnifiedFormat(n, btz_uf);
	bq[0]->ToUnifiedFormat(n, bqw_uf);
	bq[1]->ToUnifiedFormat(n, bqx_uf);
	bq[2]->ToUnifiedFormat(n, bqy_uf);
	bq[3]->ToUnifiedFormat(n, bqz_uf);

	auto *atx = (const double *)atx_uf.data;
	auto *aty = (const double *)aty_uf.data;
	auto *atz = (const double *)atz_uf.data;
	auto *aqw = (const double *)aqw_uf.data;
	auto *aqx = (const double *)aqx_uf.data;
	auto *aqy = (const double *)aqy_uf.data;
	auto *aqz = (const double *)aqz_uf.data;

	auto *btx = (const double *)btx_uf.data;
	auto *bty = (const double *)bty_uf.data;
	auto *btz = (const double *)btz_uf.data;
	auto *bqw = (const double *)bqw_uf.data;
	auto *bqx = (const double *)bqx_uf.data;
	auto *bqy = (const double *)bqy_uf.data;
	auto *bqz = (const double *)bqz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = a_uf.validity.AllValid() && b_uf.validity.AllValid() && at_uf.validity.AllValid() &&
	                       aq_uf.validity.AllValid() && bt_uf.validity.AllValid() && bq_uf.validity.AllValid() &&
	                       atx_uf.validity.AllValid() && aty_uf.validity.AllValid() && atz_uf.validity.AllValid() &&
	                       aqw_uf.validity.AllValid() && aqx_uf.validity.AllValid() && aqy_uf.validity.AllValid() &&
	                       aqz_uf.validity.AllValid() && btx_uf.validity.AllValid() && bty_uf.validity.AllValid() &&
	                       btz_uf.validity.AllValid() && bqw_uf.validity.AllValid() && bqx_uf.validity.AllValid() &&
	                       bqy_uf.validity.AllValid() && bqz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(at_uf, i) || !RowIsValid(aq_uf, i) ||
			    !RowIsValid(bt_uf, i) || !RowIsValid(bq_uf, i) || !RowIsValid(atx_uf, i) || !RowIsValid(aty_uf, i) ||
			    !RowIsValid(atz_uf, i) || !RowIsValid(aqw_uf, i) || !RowIsValid(aqx_uf, i) || !RowIsValid(aqy_uf, i) ||
			    !RowIsValid(aqz_uf, i) || !RowIsValid(btx_uf, i) || !RowIsValid(bty_uf, i) || !RowIsValid(btz_uf, i) ||
			    !RowIsValid(bqw_uf, i) || !RowIsValid(bqx_uf, i) || !RowIsValid(bqy_uf, i) || !RowIsValid(bqz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}

		const idx_t atx_i = atx_uf.sel->get_index(i);
		const idx_t aty_i = aty_uf.sel->get_index(i);
		const idx_t atz_i = atz_uf.sel->get_index(i);
		const idx_t btx_i = btx_uf.sel->get_index(i);
		const idx_t bty_i = bty_uf.sel->get_index(i);
		const idx_t btz_i = btz_uf.sel->get_index(i);

		const idx_t aqw_i = aqw_uf.sel->get_index(i);
		const idx_t aqx_i = aqx_uf.sel->get_index(i);
		const idx_t aqy_i = aqy_uf.sel->get_index(i);
		const idx_t aqz_i = aqz_uf.sel->get_index(i);

		const idx_t bqw_i = bqw_uf.sel->get_index(i);
		const idx_t bqx_i = bqx_uf.sel->get_index(i);
		const idx_t bqy_i = bqy_uf.sel->get_index(i);
		const idx_t bqz_i = bqz_uf.sel->get_index(i);

		double rtx, rty, rtz;
		QInvRotate(bqw[bqw_i], bqx[bqx_i], bqy[bqy_i], bqz[bqz_i], atx[atx_i], aty[aty_i], atz[atz_i], rtx, rty, rtz);

		otx[i] = btx[btx_i] + rtx;
		oty[i] = bty[bty_i] + rty;
		otz[i] = btz[btz_i] + rtz;

		double rw, rx, ry, rz;
		QMul(aqw[aqw_i], aqx[aqx_i], aqy[aqy_i], aqz[aqz_i], bqw[bqw_i], bqx[bqx_i], bqy[bqy_i], bqz[bqz_i], rw, rx, ry,
		     rz);
		oqw[i] = rw;
		oqx[i] = rx;
		oqy[i] = ry;
		oqz[i] = rz;
	}
}

// se3_compose(vec3, vec3) -> vec3 (translation add)
static void Se3ComposeTTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
	UnifiedVectorFormat a_uf, b_uf;
	a_v.ToUnifiedFormat(n, a_uf);
	b_v.ToUnifiedFormat(n, b_uf);

	a[0]->ToUnifiedFormat(n, ax_uf);
	a[1]->ToUnifiedFormat(n, ay_uf);
	a[2]->ToUnifiedFormat(n, az_uf);
	b[0]->ToUnifiedFormat(n, bx_uf);
	b[1]->ToUnifiedFormat(n, by_uf);
	b[2]->ToUnifiedFormat(n, bz_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = a_uf.validity.AllValid() && b_uf.validity.AllValid() && ax_uf.validity.AllValid() &&
	                       ay_uf.validity.AllValid() && az_uf.validity.AllValid() && bx_uf.validity.AllValid() &&
	                       by_uf.validity.AllValid() && bz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i) || !RowIsValid(bx_uf, i) || !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		ox[i] = ax[ax_uf.sel->get_index(i)] + bx[bx_uf.sel->get_index(i)];
		oy[i] = ay[ay_uf.sel->get_index(i)] + by[by_uf.sel->get_index(i)];
		oz[i] = az[az_uf.sel->get_index(i)] + bz[bz_uf.sel->get_index(i)];
	}
}

// se3_compose(quat, quat) -> quat (qmul)
static void Se3ComposeQQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf;
	UnifiedVectorFormat bw_uf, bx_uf, by_uf, bz_uf;
	UnifiedVectorFormat a_uf, b_uf;

	a_v.ToUnifiedFormat(n, a_uf);
	b_v.ToUnifiedFormat(n, b_uf);
	a[0]->ToUnifiedFormat(n, aw_uf);
	a[1]->ToUnifiedFormat(n, ax_uf);
	a[2]->ToUnifiedFormat(n, ay_uf);
	a[3]->ToUnifiedFormat(n, az_uf);

	b[0]->ToUnifiedFormat(n, bw_uf);
	b[1]->ToUnifiedFormat(n, bx_uf);
	b[2]->ToUnifiedFormat(n, by_uf);
	b[3]->ToUnifiedFormat(n, bz_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bw = (const double *)bw_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, ow, ox, oy, oz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = a_uf.validity.AllValid() && b_uf.validity.AllValid() && aw_uf.validity.AllValid() &&
	                       ax_uf.validity.AllValid() && ay_uf.validity.AllValid() && az_uf.validity.AllValid() &&
	                       bw_uf.validity.AllValid() && bx_uf.validity.AllValid() && by_uf.validity.AllValid() &&
	                       bz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) ||
			    !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) || !RowIsValid(bw_uf, i) || !RowIsValid(bx_uf, i) ||
			    !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		double rw, rx, ry, rz;
		QMul(aw[aw_uf.sel->get_index(i)], ax[ax_uf.sel->get_index(i)], ay[ay_uf.sel->get_index(i)],
		     az[az_uf.sel->get_index(i)], bw[bw_uf.sel->get_index(i)], bx[bx_uf.sel->get_index(i)],
		     by[by_uf.sel->get_index(i)], bz[bz_uf.sel->get_index(i)], rw, rx, ry, rz);
		ow[i] = rw;
		ox[i] = rx;
		oy[i] = ry;
		oz[i] = rz;
	}
}

// se3_compose(q, t) -> W (apply t then q)
static void Se3ComposeQTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &q_v = input.data[0];
	auto &t_v = input.data[1];
	auto &q = StructVector::GetEntries(q_v);
	auto &t = StructVector::GetEntries(t_v);

	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf, tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat q_uf, t_uf;
	q_v.ToUnifiedFormat(n, q_uf);
	t_v.ToUnifiedFormat(n, t_uf);
	q[0]->ToUnifiedFormat(n, qw_uf);
	q[1]->ToUnifiedFormat(n, qx_uf);
	q[2]->ToUnifiedFormat(n, qy_uf);
	q[3]->ToUnifiedFormat(n, qz_uf);
	t[0]->ToUnifiedFormat(n, tx_uf);
	t[1]->ToUnifiedFormat(n, ty_uf);
	t[2]->ToUnifiedFormat(n, tz_uf);

	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && t_uf.validity.AllValid() && qw_uf.validity.AllValid() &&
	                       qx_uf.validity.AllValid() && qy_uf.validity.AllValid() && qz_uf.validity.AllValid() &&
	                       tx_uf.validity.AllValid() && ty_uf.validity.AllValid() && tz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);

		otx[i] = tx[itx_i];
		oty[i] = ty[ity_i];
		otz[i] = tz[itz_i];

		oqw[i] = qw[iqw_i];
		oqx[i] = qx[iqx_i];
		oqy[i] = qy[iqy_i];
		oqz[i] = qz[iqz_i];
	}
}

// se3_compose(t, q) -> W (apply q then t)
static void Se3ComposeTQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &q_v = input.data[1];
	auto &t = StructVector::GetEntries(t_v);
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf, tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat q_uf, t_uf;
	q_v.ToUnifiedFormat(n, q_uf);
	t_v.ToUnifiedFormat(n, t_uf);
	q[0]->ToUnifiedFormat(n, qw_uf);
	q[1]->ToUnifiedFormat(n, qx_uf);
	q[2]->ToUnifiedFormat(n, qy_uf);
	q[3]->ToUnifiedFormat(n, qz_uf);
	t[0]->ToUnifiedFormat(n, tx_uf);
	t[1]->ToUnifiedFormat(n, ty_uf);
	t[2]->ToUnifiedFormat(n, tz_uf);

	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && t_uf.validity.AllValid() && qw_uf.validity.AllValid() &&
	                       qx_uf.validity.AllValid() && qy_uf.validity.AllValid() && qz_uf.validity.AllValid() &&
	                       tx_uf.validity.AllValid() && ty_uf.validity.AllValid() && tz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);

		double rtx, rty, rtz;
		QInvRotate(qw[iqw_i], qx[iqx_i], qy[iqy_i], qz[iqz_i], tx[itx_i], ty[ity_i], tz[itz_i], rtx, rty, rtz);
		otx[i] = rtx;
		oty[i] = rty;
		otz[i] = rtz;

		oqw[i] = qw[iqw_i];
		oqx[i] = qx[iqx_i];
		oqy[i] = qy[iqy_i];
		oqz[i] = qz[iqz_i];
	}
}

// se3_compose(W, q) -> W (apply q then W)
static void Se3ComposeWQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &w_v = input.data[0];
	auto &q_v = input.data[1];
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(*went[0]);
	auto &qent = StructVector::GetEntries(*went[1]);
	auto &q2ent = StructVector::GetEntries(q_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat q2w_uf, q2x_uf, q2y_uf, q2z_uf;
	UnifiedVectorFormat w_uf, t_uf, q_uf, q2_uf;

	w_v.ToUnifiedFormat(n, w_uf);
	went[0]->ToUnifiedFormat(n, t_uf);
	went[1]->ToUnifiedFormat(n, q_uf);
	q_v.ToUnifiedFormat(n, q2_uf);

	tent[0]->ToUnifiedFormat(n, tx_uf);
	tent[1]->ToUnifiedFormat(n, ty_uf);
	tent[2]->ToUnifiedFormat(n, tz_uf);
	qent[0]->ToUnifiedFormat(n, qw_uf);
	qent[1]->ToUnifiedFormat(n, qx_uf);
	qent[2]->ToUnifiedFormat(n, qy_uf);
	qent[3]->ToUnifiedFormat(n, qz_uf);

	q2ent[0]->ToUnifiedFormat(n, q2w_uf);
	q2ent[1]->ToUnifiedFormat(n, q2x_uf);
	q2ent[2]->ToUnifiedFormat(n, q2y_uf);
	q2ent[3]->ToUnifiedFormat(n, q2z_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;
	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *q2w = (const double *)q2w_uf.data;
	auto *q2x = (const double *)q2x_uf.data;
	auto *q2y = (const double *)q2y_uf.data;
	auto *q2z = (const double *)q2z_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = w_uf.validity.AllValid() && t_uf.validity.AllValid() && q_uf.validity.AllValid() &&
	                       q2_uf.validity.AllValid() && tx_uf.validity.AllValid() && ty_uf.validity.AllValid() &&
	                       tz_uf.validity.AllValid() && qw_uf.validity.AllValid() && qx_uf.validity.AllValid() &&
	                       qy_uf.validity.AllValid() && qz_uf.validity.AllValid() && q2w_uf.validity.AllValid() &&
	                       q2x_uf.validity.AllValid() && q2y_uf.validity.AllValid() && q2z_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(q2_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) ||
			    !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(q2w_uf, i) ||
			    !RowIsValid(q2x_uf, i) || !RowIsValid(q2y_uf, i) || !RowIsValid(q2z_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);
		const idx_t iq2w_i = q2w_uf.sel->get_index(i);
		const idx_t iq2x_i = q2x_uf.sel->get_index(i);
		const idx_t iq2y_i = q2y_uf.sel->get_index(i);
		const idx_t iq2z_i = q2z_uf.sel->get_index(i);

		double rtx, rty, rtz;
		QInvRotate(q2w[iq2w_i], q2x[iq2x_i], q2y[iq2y_i], q2z[iq2z_i], tx[itx_i], ty[ity_i], tz[itz_i], rtx, rty, rtz);
		otx[i] = rtx;
		oty[i] = rty;
		otz[i] = rtz;

		double rw, rx, ry, rz;
		QMul(qw[iqw_i], qx[iqx_i], qy[iqy_i], qz[iqz_i], q2w[iq2w_i], q2x[iq2x_i], q2y[iq2y_i], q2z[iq2z_i], rw, rx, ry,
		     rz);
		oqw[i] = rw;
		oqx[i] = rx;
		oqy[i] = ry;
		oqz[i] = rz;
	}
}

// se3_compose(q, W) -> W (apply W then q)
static void Se3ComposeQWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &q_v = input.data[0];
	auto &w_v = input.data[1];
	auto &qent = StructVector::GetEntries(q_v);
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(*went[0]);
	auto &wqent = StructVector::GetEntries(*went[1]);

	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, wqw_uf, wqx_uf, wqy_uf, wqz_uf;
	UnifiedVectorFormat q_uf, w_uf, t_uf, wq_uf;

	q_v.ToUnifiedFormat(n, q_uf);
	w_v.ToUnifiedFormat(n, w_uf);
	went[0]->ToUnifiedFormat(n, t_uf);
	went[1]->ToUnifiedFormat(n, wq_uf);

	qent[0]->ToUnifiedFormat(n, qw_uf);
	qent[1]->ToUnifiedFormat(n, qx_uf);
	qent[2]->ToUnifiedFormat(n, qy_uf);
	qent[3]->ToUnifiedFormat(n, qz_uf);
	tent[0]->ToUnifiedFormat(n, tx_uf);
	tent[1]->ToUnifiedFormat(n, ty_uf);
	tent[2]->ToUnifiedFormat(n, tz_uf);
	wqent[0]->ToUnifiedFormat(n, wqw_uf);
	wqent[1]->ToUnifiedFormat(n, wqx_uf);
	wqent[2]->ToUnifiedFormat(n, wqy_uf);
	wqent[3]->ToUnifiedFormat(n, wqz_uf);

	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;
	auto *wqw = (const double *)wqw_uf.data;
	auto *wqx = (const double *)wqx_uf.data;
	auto *wqy = (const double *)wqy_uf.data;
	auto *wqz = (const double *)wqz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = q_uf.validity.AllValid() && w_uf.validity.AllValid() && t_uf.validity.AllValid() &&
	                       wq_uf.validity.AllValid() && qw_uf.validity.AllValid() && qx_uf.validity.AllValid() &&
	                       qy_uf.validity.AllValid() && qz_uf.validity.AllValid() && tx_uf.validity.AllValid() &&
	                       ty_uf.validity.AllValid() && tz_uf.validity.AllValid() && wqw_uf.validity.AllValid() &&
	                       wqx_uf.validity.AllValid() && wqy_uf.validity.AllValid() && wqz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(wq_uf, i) ||
			    !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(wqw_uf, i) ||
			    !RowIsValid(wqx_uf, i) || !RowIsValid(wqy_uf, i) || !RowIsValid(wqz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);
		const idx_t iwqw_i = wqw_uf.sel->get_index(i);
		const idx_t iwqx_i = wqx_uf.sel->get_index(i);
		const idx_t iwqy_i = wqy_uf.sel->get_index(i);
		const idx_t iwqz_i = wqz_uf.sel->get_index(i);
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);

		otx[i] = tx[itx_i];
		oty[i] = ty[ity_i];
		otz[i] = tz[itz_i];

		double rw, rx, ry, rz;
		QMul(qw[iqw_i], qx[iqx_i], qy[iqy_i], qz[iqz_i], wqw[iwqw_i], wqx[iwqx_i], wqy[iwqy_i], wqz[iwqz_i], rw, rx, ry,
		     rz);
		oqw[i] = rw;
		oqx[i] = rx;
		oqy[i] = ry;
		oqz[i] = rz;
	}
}

// se3_compose(W, t) -> W (apply t then W)
static void Se3ComposeWTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &w_v = input.data[0];
	auto &t_v = input.data[1];
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(*went[0]);
	auto &qent = StructVector::GetEntries(*went[1]);
	auto &t2 = StructVector::GetEntries(t_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat t2x_uf, t2y_uf, t2z_uf;
	UnifiedVectorFormat w_uf, t_uf, q_uf, t2_uf;

	w_v.ToUnifiedFormat(n, w_uf);
	went[0]->ToUnifiedFormat(n, t_uf);
	went[1]->ToUnifiedFormat(n, q_uf);
	t_v.ToUnifiedFormat(n, t2_uf);

	tent[0]->ToUnifiedFormat(n, tx_uf);
	tent[1]->ToUnifiedFormat(n, ty_uf);
	tent[2]->ToUnifiedFormat(n, tz_uf);
	qent[0]->ToUnifiedFormat(n, qw_uf);
	qent[1]->ToUnifiedFormat(n, qx_uf);
	qent[2]->ToUnifiedFormat(n, qy_uf);
	qent[3]->ToUnifiedFormat(n, qz_uf);

	t2[0]->ToUnifiedFormat(n, t2x_uf);
	t2[1]->ToUnifiedFormat(n, t2y_uf);
	t2[2]->ToUnifiedFormat(n, t2z_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;
	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *t2x = (const double *)t2x_uf.data;
	auto *t2y = (const double *)t2y_uf.data;
	auto *t2z = (const double *)t2z_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = w_uf.validity.AllValid() && t_uf.validity.AllValid() && q_uf.validity.AllValid() &&
	                       t2_uf.validity.AllValid() && tx_uf.validity.AllValid() && ty_uf.validity.AllValid() &&
	                       tz_uf.validity.AllValid() && qw_uf.validity.AllValid() && qx_uf.validity.AllValid() &&
	                       qy_uf.validity.AllValid() && qz_uf.validity.AllValid() && t2x_uf.validity.AllValid() &&
	                       t2y_uf.validity.AllValid() && t2z_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(t2_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) ||
			    !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(t2x_uf, i) ||
			    !RowIsValid(t2y_uf, i) || !RowIsValid(t2z_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);
		const idx_t it2x_i = t2x_uf.sel->get_index(i);
		const idx_t it2y_i = t2y_uf.sel->get_index(i);
		const idx_t it2z_i = t2z_uf.sel->get_index(i);
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);

		otx[i] = tx[itx_i] + t2x[it2x_i];
		oty[i] = ty[ity_i] + t2y[it2y_i];
		otz[i] = tz[itz_i] + t2z[it2z_i];

		oqw[i] = qw[iqw_i];
		oqx[i] = qx[iqx_i];
		oqy[i] = qy[iqy_i];
		oqz[i] = qz[iqz_i];
	}
}

// se3_compose(t, W) -> W (apply W then t)
static void Se3ComposeTWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &w_v = input.data[1];
	auto &t2 = StructVector::GetEntries(t_v);
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(*went[0]);
	auto &qent = StructVector::GetEntries(*went[1]);

	UnifiedVectorFormat t2x_uf, t2y_uf, t2z_uf;
	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat t2_uf, w_uf, t_uf, q_uf;

	t_v.ToUnifiedFormat(n, t2_uf);
	w_v.ToUnifiedFormat(n, w_uf);
	went[0]->ToUnifiedFormat(n, t_uf);
	went[1]->ToUnifiedFormat(n, q_uf);

	t2[0]->ToUnifiedFormat(n, t2x_uf);
	t2[1]->ToUnifiedFormat(n, t2y_uf);
	t2[2]->ToUnifiedFormat(n, t2z_uf);

	tent[0]->ToUnifiedFormat(n, tx_uf);
	tent[1]->ToUnifiedFormat(n, ty_uf);
	tent[2]->ToUnifiedFormat(n, tz_uf);
	qent[0]->ToUnifiedFormat(n, qw_uf);
	qent[1]->ToUnifiedFormat(n, qx_uf);
	qent[2]->ToUnifiedFormat(n, qy_uf);
	qent[3]->ToUnifiedFormat(n, qz_uf);

	auto *t2x = (const double *)t2x_uf.data;
	auto *t2y = (const double *)t2y_uf.data;
	auto *t2z = (const double *)t2z_uf.data;
	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;
	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, otx, oty, otz, oqw, oqx, oqy, oqz);
	auto &out_validity = FlatVector::Validity(result);
	out_validity.SetAllValid(n);

	const bool all_valid = t2_uf.validity.AllValid() && w_uf.validity.AllValid() && t_uf.validity.AllValid() &&
	                       q_uf.validity.AllValid() && t2x_uf.validity.AllValid() && t2y_uf.validity.AllValid() &&
	                       t2z_uf.validity.AllValid() && tx_uf.validity.AllValid() && ty_uf.validity.AllValid() &&
	                       tz_uf.validity.AllValid() && qw_uf.validity.AllValid() && qx_uf.validity.AllValid() &&
	                       qy_uf.validity.AllValid() && qz_uf.validity.AllValid();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t2_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) ||
			    !RowIsValid(t2x_uf, i) || !RowIsValid(t2y_uf, i) || !RowIsValid(t2z_uf, i) || !RowIsValid(tx_uf, i) ||
			    !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const idx_t it2x_i = t2x_uf.sel->get_index(i);
		const idx_t it2y_i = t2y_uf.sel->get_index(i);
		const idx_t it2z_i = t2z_uf.sel->get_index(i);
		const idx_t itx_i = tx_uf.sel->get_index(i);
		const idx_t ity_i = ty_uf.sel->get_index(i);
		const idx_t itz_i = tz_uf.sel->get_index(i);
		const idx_t iqw_i = qw_uf.sel->get_index(i);
		const idx_t iqx_i = qx_uf.sel->get_index(i);
		const idx_t iqy_i = qy_uf.sel->get_index(i);
		const idx_t iqz_i = qz_uf.sel->get_index(i);

		double rtx, rty, rtz;
		QInvRotate(qw[iqw_i], qx[iqx_i], qy[iqy_i], qz[iqz_i], t2x[it2x_i], t2y[it2y_i], t2z[it2z_i], rtx, rty, rtz);

		otx[i] = tx[itx_i] + rtx;
		oty[i] = ty[ity_i] + rty;
		otz[i] = tz[itz_i] + rtz;

		oqw[i] = qw[iqw_i];
		oqx[i] = qx[iqx_i];
		oqy[i] = qy[iqy_i];
		oqz[i] = qz[iqz_i];
	}
}

// ------------------------- Registration -------------------------
static void LoadInternal(ExtensionLoader &loader) {
	loader.RegisterFunction(
	    ScalarFunction("quat_from_axis_angle", {Vec3Type(), LogicalType::DOUBLE}, QuatType(), QuatFromAxisAngleFn));

	loader.RegisterFunction(ScalarFunction("qmul", {QuatType(), QuatType()}, QuatType(), QMulFn));

	loader.RegisterFunction(ScalarFunction("qconj", {QuatType()}, QuatType(), QConjFn));

	loader.RegisterFunction(ScalarFunction("qnorm2", {QuatType()}, LogicalType::DOUBLE, QNorm2Fn));

	loader.RegisterFunction(ScalarFunction("se3_identity", {}, WType(), Se3IdentityFn));

	loader.RegisterFunction(ScalarFunction("se3_make", {Vec3Type(), QuatType()}, WType(), Se3MakeFn));

	loader.RegisterFunction(ScalarFunction("se3_from_axis_angle", {Vec3Type(), Vec3Type(), LogicalType::DOUBLE},
	                                       WType(), Se3FromAxisAngleFn));

	loader.RegisterFunction(ScalarFunction("se3_apply", {WType(), Vec3Type()}, Vec3Type(), Se3ApplyFn));
	loader.RegisterFunction(ScalarFunction("se3_apply", {Vec3Type(), Vec3Type()}, Vec3Type(), Se3ApplyTFn));
	loader.RegisterFunction(ScalarFunction("se3_apply", {QuatType(), Vec3Type()}, Vec3Type(), Se3ApplyQFn));

	// se3_inv overloads
	loader.RegisterFunction(ScalarFunction("se3_inv", {Vec3Type()}, Vec3Type(), Se3InvTFn));
	loader.RegisterFunction(ScalarFunction("se3_inv", {QuatType()}, QuatType(), Se3InvQFn));
	loader.RegisterFunction(ScalarFunction("se3_inv", {WType()}, WType(), Se3InvWFn));

	// se3_compose overloads (apply B then A)
	loader.RegisterFunction(ScalarFunction("se3_compose", {WType(), WType()}, WType(), Se3ComposeWWFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {Vec3Type(), Vec3Type()}, Vec3Type(), Se3ComposeTTFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {QuatType(), QuatType()}, QuatType(), Se3ComposeQQFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {QuatType(), Vec3Type()}, WType(), Se3ComposeQTFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {Vec3Type(), QuatType()}, WType(), Se3ComposeTQFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {WType(), QuatType()}, WType(), Se3ComposeWQFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {QuatType(), WType()}, WType(), Se3ComposeQWFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {WType(), Vec3Type()}, WType(), Se3ComposeWTFn));
	loader.RegisterFunction(ScalarFunction("se3_compose", {Vec3Type(), WType()}, WType(), Se3ComposeTWFn));
}

void Se3Extension::Load(ExtensionLoader &loader) {
	LoadInternal(loader);
}

std::string Se3Extension::Name() {
	return "se3";
}

std::string Se3Extension::Version() const {
#ifdef EXT_VERSION_SE3
	return EXT_VERSION_SE3;
#else
	return "";
#endif
}

} // namespace duckdb

extern "C" {

DUCKDB_CPP_EXTENSION_ENTRY(se3, loader) {
	duckdb::LoadInternal(loader);
}
}
