#include "se3_functions.hpp"
#include "se3_math.hpp"

#include "duckdb/common/vector/flat_vector.hpp"
#include "duckdb/common/vector/constant_vector.hpp"
#include "duckdb/common/vector/struct_vector.hpp"
#include "duckdb/common/vector/vector_iterator.hpp"
#include "duckdb/common/vector/vector_writer.hpp"
#include "duckdb/common/vector_operations/binary_executor.hpp"
#include "duckdb/common/vector_operations/unary_executor.hpp"

#include <functional>
#include <initializer_list>
#include <tuple>
#include <type_traits>

namespace duckdb {
namespace se3 {

// ------------------------- Validity helpers -------------------------
static inline bool RowIsValid(const UnifiedVectorFormat &uf, idx_t i) {
	return uf.validity.RowIsValid(uf.sel->get_index(i));
}

using Vec3Vector = VectorStructType<double, double, double>;
using QuatVector = VectorStructType<double, double, double, double>;
using TransformVector = VectorStructType<Vec3Vector, QuatVector>;

template <bool ALL_VALID>
static inline bool ReadVec3(const VectorIterator<Vec3Vector>::ValueEntry &entry, Vec3 &value) {
	auto x = entry.GetChildValue<0>();
	auto y = entry.GetChildValue<1>();
	auto z = entry.GetChildValue<2>();
	if (!ALL_VALID && (!entry.IsValid() || !x.IsValid() || !y.IsValid() || !z.IsValid())) {
		return false;
	}
	value = {x.GetValue(), y.GetValue(), z.GetValue()};
	return true;
}

template <bool ALL_VALID>
static inline bool ReadQuat(const VectorIterator<QuatVector>::ValueEntry &entry, Quat &value) {
	auto w = entry.GetChildValue<0>();
	auto x = entry.GetChildValue<1>();
	auto y = entry.GetChildValue<2>();
	auto z = entry.GetChildValue<3>();
	if (!ALL_VALID && (!entry.IsValid() || !w.IsValid() || !x.IsValid() || !y.IsValid() || !z.IsValid())) {
		return false;
	}
	value = {w.GetValue(), x.GetValue(), y.GetValue(), z.GetValue()};
	return true;
}

template <bool ALL_VALID>
static inline bool ReadTransform(const VectorIterator<TransformVector>::ValueEntry &entry, Vec3 &t, Quat &q) {
	if (!ALL_VALID && !entry.IsValid()) {
		return false;
	}
	return ReadVec3<ALL_VALID>(entry.GetChildValue<0>(), t) && ReadQuat<ALL_VALID>(entry.GetChildValue<1>(), q);
}

static void WriteVec3(VectorWriter<Vec3Vector> &writer, const Vec3 &value) {
	writer.WriteValue([&](auto &x, auto &y, auto &z) {
		x.WriteValue(value.x);
		y.WriteValue(value.y);
		z.WriteValue(value.z);
	});
}

static void WriteQuat(VectorWriter<QuatVector> &writer, const Quat &value) {
	writer.WriteValue([&](auto &w, auto &x, auto &y, auto &z) {
		w.WriteValue(value.w);
		x.WriteValue(value.x);
		y.WriteValue(value.y);
		z.WriteValue(value.z);
	});
}

static bool CannotHaveNullComponents(const Vector &value) {
	const auto type = value.GetVectorType();
	const bool can_have_null =
	    type == VectorType::FLAT_VECTOR ? FlatVector::Validity(value).CanHaveNull() : value.Validity().CanHaveNull();
	if (can_have_null) {
		return false;
	}
	if (value.GetType().InternalType() == PhysicalType::STRUCT) {
		for (auto &child : StructVector::GetEntries(value)) {
			if (!CannotHaveNullComponents(child)) {
				return false;
			}
		}
	}
	return true;
}

static bool IsFlatAndValid(const Vector &value) {
	if (value.GetVectorType() != VectorType::FLAT_VECTOR || FlatVector::Validity(value).CanHaveNull()) {
		return false;
	}
	if (value.GetType().InternalType() == PhysicalType::STRUCT) {
		for (const auto &child : StructVector::GetEntries(value)) {
			if (!IsFlatAndValid(child)) {
				return false;
			}
		}
	}
	return true;
}

struct FlatVec3 {
	explicit FlatVec3(const Vector &vector) {
		const auto &children = StructVector::GetEntries(vector);
		x = FlatVector::GetData<double>(children[0]);
		y = FlatVector::GetData<double>(children[1]);
		z = FlatVector::GetData<double>(children[2]);
	}
	Vec3 operator[](idx_t i) const {
		return {x[i], y[i], z[i]};
	}
	const double *x, *y, *z;
};

struct FlatQuat {
	explicit FlatQuat(const Vector &vector) {
		const auto &children = StructVector::GetEntries(vector);
		w = FlatVector::GetData<double>(children[0]);
		x = FlatVector::GetData<double>(children[1]);
		y = FlatVector::GetData<double>(children[2]);
		z = FlatVector::GetData<double>(children[3]);
	}
	Quat operator[](idx_t i) const {
		return {w[i], x[i], y[i], z[i]};
	}
	const double *w, *x, *y, *z;
};

template <class READER, class RESULT, class OP, class... INPUTS>
static bool TryExecuteFlat(Vector &result, idx_t count, OP operation, const INPUTS &...inputs) {
	if (!(IsFlatAndValid(inputs) && ...)) {
		return false;
	}
	auto readers = std::make_tuple(READER(inputs)...);
	auto writer = FlatVector::Writer<RESULT>(result, count);
	for (idx_t i = 0; i < count; i++) {
		auto value = std::apply([&](const auto &...reader) { return operation(reader[i]...); }, readers);
		if constexpr (std::is_same<RESULT, Vec3Vector>::value) {
			WriteVec3(writer, value);
		} else if constexpr (std::is_same<RESULT, QuatVector>::value) {
			WriteQuat(writer, value);
		} else {
			writer.WriteValue(value);
		}
	}
	return true;
}

// ------------------------- Output writers -------------------------
static inline void PrepareStructChildrenFlat(Vector &struct_vec, idx_t count) {
	struct_vec.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(struct_vec, count);
}

// vec3 result: STRUCT(x,y,z)
static inline void PrepareVec3Out(Vector &result, idx_t count, double *&ox, double *&oy, double *&oz) {
	PrepareStructChildrenFlat(result, count);
	auto &ent = StructVector::GetEntries(result);
	ox = FlatVector::GetDataMutable<double>(ent[0]);
	oy = FlatVector::GetDataMutable<double>(ent[1]);
	oz = FlatVector::GetDataMutable<double>(ent[2]);
}

// quat result: STRUCT(w,x,y,z)
static inline void PrepareQuatOut(Vector &result, idx_t count, double *&ow, double *&ox, double *&oy, double *&oz) {
	PrepareStructChildrenFlat(result, count);
	auto &ent = StructVector::GetEntries(result);
	ow = FlatVector::GetDataMutable<double>(ent[0]);
	ox = FlatVector::GetDataMutable<double>(ent[1]);
	oy = FlatVector::GetDataMutable<double>(ent[2]);
	oz = FlatVector::GetDataMutable<double>(ent[3]);
}

// W result: STRUCT(t vec3, q quat)
static inline void PrepareWOut(Vector &result, idx_t count, double *&tx, double *&ty, double *&tz, double *&qw,
                               double *&qx, double *&qy, double *&qz) {
	PrepareStructChildrenFlat(result, count);
	auto &went = StructVector::GetEntries(result);

	// t
	auto &tent = StructVector::GetEntries(went[0]);
	tx = FlatVector::GetDataMutable<double>(tent[0]);
	ty = FlatVector::GetDataMutable<double>(tent[1]);
	tz = FlatVector::GetDataMutable<double>(tent[2]);

	// q
	auto &qent = StructVector::GetEntries(went[1]);
	qw = FlatVector::GetDataMutable<double>(qent[0]);
	qx = FlatVector::GetDataMutable<double>(qent[1]);
	qy = FlatVector::GetDataMutable<double>(qent[2]);
	qz = FlatVector::GetDataMutable<double>(qent[3]);
}

// ------------------------- Functions -------------------------

static bool TryReferenceStruct(Vector &result, std::initializer_list<const Vector *> inputs) {
	for (const auto *input : inputs) {
		if (!CannotHaveNullComponents(*input)) {
			return false;
		}
	}
	auto &children = StructVector::GetEntries(result);
	bool all_constant = true;
	idx_t count = 0;
	idx_t child = 0;
	for (const auto *input : inputs) {
		all_constant &= input->GetVectorType() == VectorType::CONSTANT_VECTOR;
		children[child++].Reference(*input);
		count = MaxValue<idx_t>(count, input->size());
	}
	result.BufferMutable().SetVectorTypeOnly(all_constant ? VectorType::CONSTANT_VECTOR : VectorType::FLAT_VECTOR);
	result.BufferMutable().SetVectorSizeOnly(count);
	return true;
}

template <class OP>
static bool TryExecuteComponentwise(const Vector &a, const Vector &b, Vector &result, idx_t count, OP operation) {
	if (!CannotHaveNullComponents(a) || !CannotHaveNullComponents(b)) {
		return false;
	}
	const auto &left = StructVector::GetEntries(a);
	const auto *right = b.GetType().InternalType() == PhysicalType::STRUCT ? &StructVector::GetEntries(b) : nullptr;
	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, count);
	auto &output = StructVector::GetEntries(result);
	for (idx_t child = 0; child < output.size(); child++) {
		BinaryExecutor::Execute<double, double, double>(left[child], right ? (*right)[child] : b, output[child], count,
		                                                operation);
	}
	return true;
}

// Conjugation preserves the scalar component; translation inversion negates all components.
static bool TryNegateComponents(const Vector &input, Vector &result, idx_t count, idx_t first_negated) {
	if (!CannotHaveNullComponents(input)) {
		return false;
	}
	PrepareStructChildrenFlat(result, count);
	const auto &source = StructVector::GetEntries(input);
	auto &target = StructVector::GetEntries(result);
	for (idx_t component = 0; component < target.size(); component++) {
		if (component < first_negated) {
			target[component].Reference(source[component]);
		} else {
			UnaryExecutor::Execute<double, double>(source[component], target[component], count,
			                                       [](double value) { return -value; });
		}
	}
	return true;
}

// vvec(x,y,z) -> vec3
void Vec3CtorFn(DataChunk &input, ExpressionState &, Vector &result) {
	if (TryReferenceStruct(result, {&input.data[0], &input.data[1], &input.data[2]})) {
		return;
	}
	const idx_t n = input.size();

	UnifiedVectorFormat x_uf, y_uf, z_uf;
	input.data[0].ToUnifiedFormat(x_uf);
	input.data[1].ToUnifiedFormat(y_uf);
	input.data[2].ToUnifiedFormat(z_uf);

	auto *x = (const double *)x_uf.data;
	auto *y = (const double *)y_uf.data;
	auto *z = (const double *)z_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	const bool all_valid =
	    x_uf.validity.CannotHaveNull() && y_uf.validity.CannotHaveNull() && z_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(x_uf, i) || !RowIsValid(y_uf, i) || !RowIsValid(z_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}
		ox[i] = x[x_uf.sel->get_index(i)];
		oy[i] = y[y_uf.sel->get_index(i)];
		oz[i] = z[z_uf.sel->get_index(i)];
	}
}

// vvec(w,x,y,z) -> vec4 (struct layout compatible with quat)
void Vec4CtorFn(DataChunk &input, ExpressionState &, Vector &result) {
	if (TryReferenceStruct(result, {&input.data[0], &input.data[1], &input.data[2], &input.data[3]})) {
		return;
	}
	const idx_t n = input.size();

	UnifiedVectorFormat w_uf, x_uf, y_uf, z_uf;
	input.data[0].ToUnifiedFormat(w_uf);
	input.data[1].ToUnifiedFormat(x_uf);
	input.data[2].ToUnifiedFormat(y_uf);
	input.data[3].ToUnifiedFormat(z_uf);

	auto *w = (const double *)w_uf.data;
	auto *x = (const double *)x_uf.data;
	auto *y = (const double *)y_uf.data;
	auto *z = (const double *)z_uf.data;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	const bool all_valid = w_uf.validity.CannotHaveNull() && x_uf.validity.CannotHaveNull() &&
	                       y_uf.validity.CannotHaveNull() && z_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(x_uf, i) || !RowIsValid(y_uf, i) || !RowIsValid(z_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}
		ow[i] = w[w_uf.sel->get_index(i)];
		ox[i] = x[x_uf.sel->get_index(i)];
		oy[i] = y[y_uf.sel->get_index(i)];
		oz[i] = z[z_uf.sel->get_index(i)];
	}
}

template <class OP>
static void ExecuteVec3Binary(const Vector &a, const Vector &b, Vector &result, idx_t count, OP operation) {
	if (TryExecuteComponentwise(a, b, result, count, operation)) {
		return;
	}
	auto left = a.Values<Vec3Vector>();
	auto right = b.Values<Vec3Vector>();
	auto writer = FlatVector::Writer<Vec3Vector>(result, count);
	for (idx_t i = 0; i < count; i++) {
		Vec3 a_value, b_value;
		if (!ReadVec3<false>(left[i], a_value) || !ReadVec3<false>(right[i], b_value)) {
			writer.WriteNull();
			continue;
		}
		WriteVec3(writer,
		          {operation(a_value.x, b_value.x), operation(a_value.y, b_value.y), operation(a_value.z, b_value.z)});
	}
}

template <class OP>
static void ExecuteQuatBinary(const Vector &a, const Vector &b, Vector &result, idx_t count, OP operation) {
	if (TryExecuteComponentwise(a, b, result, count, operation)) {
		return;
	}
	auto left = a.Values<QuatVector>();
	auto right = b.Values<QuatVector>();
	auto writer = FlatVector::Writer<QuatVector>(result, count);
	for (idx_t i = 0; i < count; i++) {
		Quat a_value, b_value;
		if (!ReadQuat<false>(left[i], a_value) || !ReadQuat<false>(right[i], b_value)) {
			writer.WriteNull();
			continue;
		}
		WriteQuat(writer, {operation(a_value.w, b_value.w), operation(a_value.x, b_value.x),
		                   operation(a_value.y, b_value.y), operation(a_value.z, b_value.z)});
	}
}

static void AddVec3(const Vector &a, const Vector &b, Vector &result, idx_t count) {
	ExecuteVec3Binary(a, b, result, count, std::plus<double>());
}

void VAddVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	AddVec3(input.data[0], input.data[1], result, input.size());
}

void VAddVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	ExecuteQuatBinary(input.data[0], input.data[1], result, input.size(), std::plus<double>());
}

void VSubVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	ExecuteVec3Binary(input.data[0], input.data[1], result, input.size(), std::minus<double>());
}

void VSubVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	ExecuteQuatBinary(input.data[0], input.data[1], result, input.size(), std::minus<double>());
}

void VScaleVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t count = input.size();
	if (TryExecuteComponentwise(input.data[0], input.data[1], result, count, std::multiplies<double>())) {
		return;
	}
	auto values = input.data[0].Values<Vec3Vector>();
	auto scales = input.data[1].Values<double>();
	auto writer = FlatVector::Writer<Vec3Vector>(result, count);
	for (idx_t i = 0; i < count; i++) {
		Vec3 value;
		auto scalar = scales[i];
		if (!ReadVec3<false>(values[i], value) || !scalar.IsValid()) {
			writer.WriteNull();
			continue;
		}
		const double scale = scalar.GetValue();
		WriteVec3(writer, {value.x * scale, value.y * scale, value.z * scale});
	}
}

void VScaleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t count = input.size();
	if (TryExecuteComponentwise(input.data[0], input.data[1], result, count, std::multiplies<double>())) {
		return;
	}
	auto values = input.data[0].Values<QuatVector>();
	auto scales = input.data[1].Values<double>();
	auto writer = FlatVector::Writer<QuatVector>(result, count);
	for (idx_t i = 0; i < count; i++) {
		Quat value;
		auto scalar = scales[i];
		if (!ReadQuat<false>(values[i], value) || !scalar.IsValid()) {
			writer.WriteNull();
			continue;
		}
		const double scale = scalar.GetValue();
		WriteQuat(writer, {value.w * scale, value.x * scale, value.y * scale, value.z * scale});
	}
}

// vdot(vec3, vec3) -> DOUBLE
void VDotVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatVec3, double>(
	        result, n, [](Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }, input.data[0],
	        input.data[1])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bx_uf);
	b[1].ToUnifiedFormat(by_uf);
	b[2].ToUnifiedFormat(bz_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &out_validity = FlatVector::ValidityMutable(result);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i) || !RowIsValid(bx_uf, i) || !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		out[i] = ax[ax_uf.sel->get_index(i)] * bx[bx_uf.sel->get_index(i)] +
		         ay[ay_uf.sel->get_index(i)] * by[by_uf.sel->get_index(i)] +
		         az[az_uf.sel->get_index(i)] * bz[bz_uf.sel->get_index(i)];
	}
}

// vdot(vec4, vec4) -> DOUBLE
void VDotVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatQuat, double>(
	        result, n, [](Quat a, Quat b) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }, input.data[0],
	        input.data[1])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, aw_uf, ax_uf, ay_uf, az_uf, bw_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(aw_uf);
	a[1].ToUnifiedFormat(ax_uf);
	a[2].ToUnifiedFormat(ay_uf);
	a[3].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bw_uf);
	b[1].ToUnifiedFormat(bx_uf);
	b[2].ToUnifiedFormat(by_uf);
	b[3].ToUnifiedFormat(bz_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bw = (const double *)bw_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &out_validity = FlatVector::ValidityMutable(result);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       aw_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull() &&
	                       bw_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) ||
			    !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) || !RowIsValid(bw_uf, i) || !RowIsValid(bx_uf, i) ||
			    !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		out[i] = aw[aw_uf.sel->get_index(i)] * bw[bw_uf.sel->get_index(i)] +
		         ax[ax_uf.sel->get_index(i)] * bx[bx_uf.sel->get_index(i)] +
		         ay[ay_uf.sel->get_index(i)] * by[by_uf.sel->get_index(i)] +
		         az[az_uf.sel->get_index(i)] * bz[bz_uf.sel->get_index(i)];
	}
}

// vnorm2(vec3) -> DOUBLE
void VNorm2Vec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatVec3, double>(
	        result, n, [](Vec3 a) { return a.x * a.x + a.y * a.y + a.z * a.z; }, input.data[0])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat a_uf, ax_uf, ay_uf, az_uf;
	a_v.ToUnifiedFormat(a_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &out_validity = FlatVector::ValidityMutable(result);

	const bool all_valid = a_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double x = ax[ax_uf.sel->get_index(i)];
		const double y = ay[ay_uf.sel->get_index(i)];
		const double z = az[az_uf.sel->get_index(i)];
		out[i] = x * x + y * y + z * z;
	}
}

// vnorm2(vec4) -> DOUBLE
void VNorm2Vec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatQuat, double>(
	        result, n, [](Quat a) { return a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z; }, input.data[0])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat a_uf, aw_uf, ax_uf, ay_uf, az_uf;
	a_v.ToUnifiedFormat(a_uf);
	a[0].ToUnifiedFormat(aw_uf);
	a[1].ToUnifiedFormat(ax_uf);
	a[2].ToUnifiedFormat(ay_uf);
	a[3].ToUnifiedFormat(az_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &out_validity = FlatVector::ValidityMutable(result);

	const bool all_valid = a_uf.validity.CannotHaveNull() && aw_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double w = aw[aw_uf.sel->get_index(i)];
		const double x = ax[ax_uf.sel->get_index(i)];
		const double y = ay[ay_uf.sel->get_index(i)];
		const double z = az[az_uf.sel->get_index(i)];
		out[i] = w * w + x * x + y * y + z * z;
	}
}

// vnorm(vec3) -> DOUBLE
void VNormVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatVec3, double>(
	        result, n, [](Vec3 a) { return Norm3(a.x, a.y, a.z); }, input.data[0])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat a_uf, ax_uf, ay_uf, az_uf;
	a_v.ToUnifiedFormat(a_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &out_validity = FlatVector::ValidityMutable(result);

	const bool all_valid = a_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double x = ax[ax_uf.sel->get_index(i)];
		const double y = ay[ay_uf.sel->get_index(i)];
		const double z = az[az_uf.sel->get_index(i)];
		out[i] = Norm3(x, y, z);
	}
}

// vnorm(vec4) -> DOUBLE
void VNormVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatQuat, double>(
	        result, n, [](Quat a) { return Norm4(a.w, a.x, a.y, a.z); }, input.data[0])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat a_uf, aw_uf, ax_uf, ay_uf, az_uf;
	a_v.ToUnifiedFormat(a_uf);
	a[0].ToUnifiedFormat(aw_uf);
	a[1].ToUnifiedFormat(ax_uf);
	a[2].ToUnifiedFormat(ay_uf);
	a[3].ToUnifiedFormat(az_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &out_validity = FlatVector::ValidityMutable(result);

	const bool all_valid = a_uf.validity.CannotHaveNull() && aw_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i)) {
				out_validity.SetInvalid(i);
				continue;
			}
		}
		const double w = aw[aw_uf.sel->get_index(i)];
		const double x = ax[ax_uf.sel->get_index(i)];
		const double y = ay[ay_uf.sel->get_index(i)];
		const double z = az[az_uf.sel->get_index(i)];
		out[i] = Norm4(w, x, y, z);
	}
}

// vnormalize(vec3) -> vec3
void VNormalizeVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatVec3, Vec3Vector>(
	        result, n,
	        [](Vec3 a) {
		        Vec3 normalized;
		        Normalize3(a.x, a.y, a.z, normalized.x, normalized.y, normalized.z);
		        return normalized;
	        },
	        input.data[0])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat a_uf, ax_uf, ay_uf, az_uf;
	a_v.ToUnifiedFormat(a_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double x = ax[ax_uf.sel->get_index(i)];
		const double y = ay[ay_uf.sel->get_index(i)];
		const double z = az[az_uf.sel->get_index(i)];
		Normalize3(x, y, z, ox[i], oy[i], oz[i]);
	}
}

// vnormalize(vec4) -> vec4
void VNormalizeVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatQuat, QuatVector>(
	        result, n,
	        [](Quat a) {
		        Quat normalized;
		        Normalize4(a.w, a.x, a.y, a.z, normalized.w, normalized.x, normalized.y, normalized.z);
		        return normalized;
	        },
	        input.data[0])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat a_uf, aw_uf, ax_uf, ay_uf, az_uf;
	a_v.ToUnifiedFormat(a_uf);
	a[0].ToUnifiedFormat(aw_uf);
	a[1].ToUnifiedFormat(ax_uf);
	a[2].ToUnifiedFormat(ay_uf);
	a[3].ToUnifiedFormat(az_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && aw_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double w = aw[aw_uf.sel->get_index(i)];
		const double x = ax[ax_uf.sel->get_index(i)];
		const double y = ay[ay_uf.sel->get_index(i)];
		const double z = az[az_uf.sel->get_index(i)];
		Normalize4(w, x, y, z, ow[i], ox[i], oy[i], oz[i]);
	}
}

// vcross(vec3, vec3) -> vec3
void VCrossVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteFlat<FlatVec3, Vec3Vector>(
	        result, n,
	        [](Vec3 a, Vec3 b) {
		        return Vec3 {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
	        },
	        input.data[0], input.data[1])) {
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bx_uf);
	b[1].ToUnifiedFormat(by_uf);
	b[2].ToUnifiedFormat(bz_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i) || !RowIsValid(bx_uf, i) || !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double axv = ax[ax_uf.sel->get_index(i)];
		const double ayv = ay[ay_uf.sel->get_index(i)];
		const double azv = az[az_uf.sel->get_index(i)];
		const double bxv = bx[bx_uf.sel->get_index(i)];
		const double byv = by[by_uf.sel->get_index(i)];
		const double bzv = bz[bz_uf.sel->get_index(i)];

		ox[i] = ayv * bzv - azv * byv;
		oy[i] = azv * bxv - axv * bzv;
		oz[i] = axv * byv - ayv * bxv;
	}
}

template <std::size_t N, AngleFunction FUNCTION>
static void VAngleFn(DataChunk &input, Vector &result) {
	const idx_t count = input.size();
	if constexpr (N == 3) {
		if (TryExecuteFlat<FlatVec3, double>(
		        result, count,
		        [](Vec3 a, Vec3 b) {
			        return VectorAngle<3, FUNCTION>({a.x, a.y, a.z}, {b.x, b.y, b.z});
		        },
		        input.data[0], input.data[1])) {
			return;
		}
	} else {
		if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
			const FlatQuat a(input.data[0]), b(input.data[1]);
			auto writer = FlatVector::Writer<double>(result, count);
			for (idx_t i = 0; i < count; i++) {
				const auto av = a[i], bv = b[i];
				writer.WriteValue(VectorAngle<4, FUNCTION>({av.w, av.x, av.y, av.z}, {bv.w, bv.x, bv.y, bv.z}));
			}
			return;
		}
	}
	auto &a_vector = input.data[0];
	auto &b_vector = input.data[1];
	auto &a_children = StructVector::GetEntries(a_vector);
	auto &b_children = StructVector::GetEntries(b_vector);
	UnifiedVectorFormat a_parent, b_parent;
	a_vector.ToUnifiedFormat(a_parent);
	b_vector.ToUnifiedFormat(b_parent);
	std::array<UnifiedVectorFormat, N> a_formats, b_formats;
	std::array<const double *, N> a_data, b_data;
	bool all_valid = a_parent.validity.CannotHaveNull() && b_parent.validity.CannotHaveNull();
	for (std::size_t component = 0; component < N; component++) {
		a_children[component].ToUnifiedFormat(a_formats[component]);
		b_children[component].ToUnifiedFormat(b_formats[component]);
		a_data[component] = UnifiedVectorFormat::GetData<double>(a_formats[component]);
		b_data[component] = UnifiedVectorFormat::GetData<double>(b_formats[component]);
		all_valid &= a_formats[component].validity.CannotHaveNull() && b_formats[component].validity.CannotHaveNull();
	}

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, count);
	auto *out = FlatVector::GetDataMutable<double>(result);
	auto &validity = FlatVector::ValidityMutable(result);
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid) {
			bool valid = RowIsValid(a_parent, i) && RowIsValid(b_parent, i);
			for (std::size_t component = 0; component < N; component++) {
				valid &= RowIsValid(a_formats[component], i) && RowIsValid(b_formats[component], i);
			}
			if (!valid) {
				validity.SetInvalid(i);
				continue;
			}
		}
		std::array<double, N> a, b;
		for (std::size_t component = 0; component < N; component++) {
			a[component] = a_data[component][a_formats[component].sel->get_index(i)];
			b[component] = b_data[component][b_formats[component].sel->get_index(i)];
		}
		out[i] = VectorAngle<N, FUNCTION>(a, b);
	}
}

void VCosAngleVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<3, AngleFunction::COSINE>(input, result);
}

void VCosAngleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<4, AngleFunction::COSINE>(input, result);
}

void VSinAngleVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<3, AngleFunction::SINE>(input, result);
}

void VSinAngleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<4, AngleFunction::SINE>(input, result);
}

void VTanAngleVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<3, AngleFunction::TANGENT>(input, result);
}

void VTanAngleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<4, AngleFunction::TANGENT>(input, result);
}

void VCotAngleVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<3, AngleFunction::COTANGENT>(input, result);
}

void VCotAngleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<4, AngleFunction::COTANGENT>(input, result);
}

// vangle(vec3, vec3) -> DOUBLE
void VAngleVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<3, AngleFunction::ANGLE>(input, result);
}

// vangle(vec4, vec4) -> DOUBLE
void VAngleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	VAngleFn<4, AngleFunction::ANGLE>(input, result);
}

// vproj(vec3, vec3) -> vec3
void VProjVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		const FlatVec3 a(input.data[0]), b(input.data[1]);
		double *ox, *oy, *oz;
		PrepareVec3Out(result, n, ox, oy, oz);
		for (idx_t i = 0; i < n; i++) {
			Projection3(a.x[i], a.y[i], a.z[i], b.x[i], b.y[i], b.z[i], ox[i], oy[i], oz[i]);
		}
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bx_uf);
	b[1].ToUnifiedFormat(by_uf);
	b[2].ToUnifiedFormat(bz_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i) || !RowIsValid(bx_uf, i) || !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double axv = ax[ax_uf.sel->get_index(i)];
		const double ayv = ay[ay_uf.sel->get_index(i)];
		const double azv = az[az_uf.sel->get_index(i)];
		const double bxv = bx[bx_uf.sel->get_index(i)];
		const double byv = by[by_uf.sel->get_index(i)];
		const double bzv = bz[bz_uf.sel->get_index(i)];
		Projection3(axv, ayv, azv, bxv, byv, bzv, ox[i], oy[i], oz[i]);
	}
}

// vproj(vec4, vec4) -> vec4
void VProjVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, aw_uf, ax_uf, ay_uf, az_uf, bw_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(aw_uf);
	a[1].ToUnifiedFormat(ax_uf);
	a[2].ToUnifiedFormat(ay_uf);
	a[3].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bw_uf);
	b[1].ToUnifiedFormat(bx_uf);
	b[2].ToUnifiedFormat(by_uf);
	b[3].ToUnifiedFormat(bz_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bw = (const double *)bw_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       aw_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull() &&
	                       bw_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) ||
			    !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) || !RowIsValid(bw_uf, i) || !RowIsValid(bx_uf, i) ||
			    !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double awv = aw[aw_uf.sel->get_index(i)];
		const double axv = ax[ax_uf.sel->get_index(i)];
		const double ayv = ay[ay_uf.sel->get_index(i)];
		const double azv = az[az_uf.sel->get_index(i)];
		const double bwv = bw[bw_uf.sel->get_index(i)];
		const double bxv = bx[bx_uf.sel->get_index(i)];
		const double byv = by[by_uf.sel->get_index(i)];
		const double bzv = bz[bz_uf.sel->get_index(i)];
		Projection4(awv, axv, ayv, azv, bwv, bxv, byv, bzv, ow[i], ox[i], oy[i], oz[i]);
	}
}

// vrej(vec3, vec3) -> vec3
void VRejVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bx_uf);
	b[1].ToUnifiedFormat(by_uf);
	b[2].ToUnifiedFormat(bz_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       ax_uf.validity.CannotHaveNull() && ay_uf.validity.CannotHaveNull() &&
	                       az_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) ||
			    !RowIsValid(az_uf, i) || !RowIsValid(bx_uf, i) || !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double axv = ax[ax_uf.sel->get_index(i)];
		const double ayv = ay[ay_uf.sel->get_index(i)];
		const double azv = az[az_uf.sel->get_index(i)];
		const double bxv = bx[bx_uf.sel->get_index(i)];
		const double byv = by[by_uf.sel->get_index(i)];
		const double bzv = bz[bz_uf.sel->get_index(i)];
		Projection3<true>(axv, ayv, azv, bxv, byv, bzv, ox[i], oy[i], oz[i]);
	}
}

// vrej(vec4, vec4) -> vec4
void VRejVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat a_uf, b_uf, aw_uf, ax_uf, ay_uf, az_uf, bw_uf, bx_uf, by_uf, bz_uf;
	a_v.ToUnifiedFormat(a_uf);
	b_v.ToUnifiedFormat(b_uf);
	a[0].ToUnifiedFormat(aw_uf);
	a[1].ToUnifiedFormat(ax_uf);
	a[2].ToUnifiedFormat(ay_uf);
	a[3].ToUnifiedFormat(az_uf);
	b[0].ToUnifiedFormat(bw_uf);
	b[1].ToUnifiedFormat(bx_uf);
	b[2].ToUnifiedFormat(by_uf);
	b[3].ToUnifiedFormat(bz_uf);

	auto *aw = (const double *)aw_uf.data;
	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;
	auto *bw = (const double *)bw_uf.data;
	auto *bx = (const double *)bx_uf.data;
	auto *by = (const double *)by_uf.data;
	auto *bz = (const double *)bz_uf.data;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	const bool all_valid = a_uf.validity.CannotHaveNull() && b_uf.validity.CannotHaveNull() &&
	                       aw_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull() &&
	                       bw_uf.validity.CannotHaveNull() && bx_uf.validity.CannotHaveNull() &&
	                       by_uf.validity.CannotHaveNull() && bz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(a_uf, i) || !RowIsValid(b_uf, i) || !RowIsValid(aw_uf, i) || !RowIsValid(ax_uf, i) ||
			    !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) || !RowIsValid(bw_uf, i) || !RowIsValid(bx_uf, i) ||
			    !RowIsValid(by_uf, i) || !RowIsValid(bz_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}

		const double awv = aw[aw_uf.sel->get_index(i)];
		const double axv = ax[ax_uf.sel->get_index(i)];
		const double ayv = ay[ay_uf.sel->get_index(i)];
		const double azv = az[az_uf.sel->get_index(i)];
		const double bwv = bw[bw_uf.sel->get_index(i)];
		const double bxv = bx[bx_uf.sel->get_index(i)];
		const double byv = by[by_uf.sel->get_index(i)];
		const double bzv = bz[bz_uf.sel->get_index(i)];
		Projection4<true>(awv, axv, ayv, azv, bwv, bxv, byv, bzv, ow[i], ox[i], oy[i], oz[i]);
	}
}

// Constant arguments are evaluated once per chunk, including NULL and zero axes.
template <bool CONSTANT_AXIS, bool CONSTANT_ANGLE>
static void AxisAngleQuaternion(const Vector &axis, const Vector &angle, Vector &result, idx_t count) {
	auto axes = axis.Values<Vec3Vector>();
	auto angles = angle.Values<double>();
	auto read_axis = [&](idx_t i, Vec3 &unit) {
		Vec3 value;
		if (!ReadVec3<false>(axes[i], value) || (value.x == 0.0 && value.y == 0.0 && value.z == 0.0)) {
			return false;
		}
		Normalize3(value.x, value.y, value.z, unit.x, unit.y, unit.z);
		return true;
	};
	auto read_angle = [&](idx_t i, double &sine, double &cosine) {
		auto entry = angles[i];
		if (!entry.IsValid()) {
			return false;
		}
		const double half = 0.5 * entry.GetValue();
		sine = std::sin(half);
		cosine = std::cos(half);
		return true;
	};
	Vec3 unit;
	double sine, cosine;
	if constexpr (CONSTANT_AXIS) {
		if (!read_axis(0, unit)) {
			ConstantVector::SetNull(result, count_t(count));
			return;
		}
	}
	if constexpr (CONSTANT_ANGLE) {
		if (!read_angle(0, sine, cosine)) {
			ConstantVector::SetNull(result, count_t(count));
			return;
		}
	}
	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, count, ow, ox, oy, oz);
	const idx_t iterations = CONSTANT_AXIS && CONSTANT_ANGLE ? 1 : count;
	for (idx_t i = 0; i < iterations; i++) {
		if ((!CONSTANT_AXIS && !read_axis(i, unit)) || (!CONSTANT_ANGLE && !read_angle(i, sine, cosine))) {
			FlatVector::SetNull(result, i, true);
			continue;
		}
		ow[i] = cosine;
		ox[i] = unit.x * sine;
		oy[i] = unit.y * sine;
		oz[i] = unit.z * sine;
	}
	if constexpr (CONSTANT_AXIS && CONSTANT_ANGLE) {
		result.SetVectorType(VectorType::CONSTANT_VECTOR);
	}
}

static void AxisAngleQuaternion(const Vector &axis, const Vector &angle, Vector &result, idx_t count) {
	if (axis.GetVectorType() == VectorType::CONSTANT_VECTOR) {
		if (angle.GetVectorType() == VectorType::CONSTANT_VECTOR) {
			AxisAngleQuaternion<true, true>(axis, angle, result, count);
		} else {
			AxisAngleQuaternion<true, false>(axis, angle, result, count);
		}
		return;
	}
	if (angle.GetVectorType() == VectorType::CONSTANT_VECTOR) {
		AxisAngleQuaternion<false, true>(axis, angle, result, count);
		return;
	}

	const idx_t n = count;

	auto &axis_v = axis;
	auto &axis_ent = StructVector::GetEntries(axis_v);
	auto &th_v = angle;

	UnifiedVectorFormat axis_uf, ax_uf, ay_uf, az_uf, th_uf;
	axis_v.ToUnifiedFormat(axis_uf);
	axis_ent[0].ToUnifiedFormat(ax_uf);
	axis_ent[1].ToUnifiedFormat(ay_uf);
	axis_ent[2].ToUnifiedFormat(az_uf);
	th_v.ToUnifiedFormat(th_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ax_sel = ax_uf.sel;
	auto *ay = (const double *)ay_uf.data;
	auto *ay_sel = ay_uf.sel;
	auto *az = (const double *)az_uf.data;
	auto *az_sel = az_uf.sel;
	auto *th = (const double *)th_uf.data;
	auto *th_sel = th_uf.sel;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	const bool all_valid = axis_uf.validity.CannotHaveNull() && ax_uf.validity.CannotHaveNull() &&
	                       ay_uf.validity.CannotHaveNull() && az_uf.validity.CannotHaveNull() &&
	                       th_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(axis_uf, i) || !RowIsValid(ax_uf, i) || !RowIsValid(ay_uf, i) || !RowIsValid(az_uf, i) ||
			    !RowIsValid(th_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}
		const double thv = th[th_sel->get_index(i)];
		const double axv = ax[ax_sel->get_index(i)];
		const double ayv = ay[ay_sel->get_index(i)];
		const double azv = az[az_sel->get_index(i)];
		if (!AxisAngleToQuat(axv, ayv, azv, thv, ow[i], ox[i], oy[i], oz[i])) {
			FlatVector::SetNull(result, i, true);
			continue;
		}
	}
}

// quat_from_axis_angle(axis:vec3, th:DOUBLE) -> quat
void QuatFromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result) {
	AxisAngleQuaternion(input.data[0], input.data[1], result, input.size());
}

static Quat MultiplyQuaternions(Quat a, Quat b) {
	Quat result;
	QMul(a.w, a.x, a.y, a.z, b.w, b.x, b.y, b.z, result.w, result.x, result.y, result.z);
	return result;
}

template <bool ALL_VALID>
static void MultiplyQuaternionRows(const Vector &a, const Vector &b, Vector &result, idx_t count) {
	auto left = a.Values<QuatVector>();
	auto right = b.Values<QuatVector>();
	auto writer = FlatVector::Writer<QuatVector>(result, count);
	for (idx_t i = 0; i < count; i++) {
		Quat av, bv;
		if (!ReadQuat<ALL_VALID>(left[i], av) || !ReadQuat<ALL_VALID>(right[i], bv)) {
			writer.WriteNull();
			continue;
		}
		WriteQuat(writer, MultiplyQuaternions(av, bv));
	}
}

static void MultiplyQuaternions(const Vector &a, const Vector &b, Vector &result, idx_t count) {
	if (TryExecuteFlat<FlatQuat, QuatVector>(
	        result, count, [](Quat av, Quat bv) { return MultiplyQuaternions(av, bv); }, a, b)) {
		return;
	}
	if (CannotHaveNullComponents(a) && CannotHaveNullComponents(b)) {
		MultiplyQuaternionRows<true>(a, b, result, count);
	} else {
		MultiplyQuaternionRows<false>(a, b, result, count);
	}
}

// qmul(qA,qB) -> quat
void QMulFn(DataChunk &input, ExpressionState &, Vector &result) {
	MultiplyQuaternions(input.data[0], input.data[1], result, input.size());
}

// qconj(q) -> quat
void QConjFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryNegateComponents(input.data[0], result, n, 1)) {
		return;
	}
	auto &q_v = input.data[0];
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat w_uf, x_uf, y_uf, z_uf;
	UnifiedVectorFormat q_uf;
	q_v.ToUnifiedFormat(q_uf);
	q[0].ToUnifiedFormat(w_uf);
	q[1].ToUnifiedFormat(x_uf);
	q[2].ToUnifiedFormat(y_uf);
	q[3].ToUnifiedFormat(z_uf);

	auto *w = (const double *)w_uf.data;
	auto *w_sel = w_uf.sel;
	auto *x = (const double *)x_uf.data;
	auto *x_sel = x_uf.sel;
	auto *y = (const double *)y_uf.data;
	auto *y_sel = y_uf.sel;
	auto *z = (const double *)z_uf.data;
	auto *z_sel = z_uf.sel;

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	const bool all_valid = q_uf.validity.CannotHaveNull() && w_uf.validity.CannotHaveNull() &&
	                       x_uf.validity.CannotHaveNull() && y_uf.validity.CannotHaveNull() &&
	                       z_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(x_uf, i) || !RowIsValid(y_uf, i) ||
			    !RowIsValid(z_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}
		ow[i] = w[w_sel->get_index(i)];
		ox[i] = -x[x_sel->get_index(i)];
		oy[i] = -y[y_sel->get_index(i)];
		oz[i] = -z[z_sel->get_index(i)];
	}
}

// se3_identity() -> W
void Se3IdentityFn(DataChunk &input, ExpressionState &, Vector &result) {
	(void)input;
	const idx_t n = input.size();

	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);

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

static void MakeTransform(const Vector &t_v, const Vector &q_v, Vector &result, idx_t n) {
	if (TryReferenceStruct(result, {&t_v, &q_v})) {
		return;
	}
	auto &t = StructVector::GetEntries(t_v);
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat t_uf, q_uf;

	t_v.ToUnifiedFormat(t_uf);
	q_v.ToUnifiedFormat(q_uf);
	t[0].ToUnifiedFormat(tx_uf);
	t[1].ToUnifiedFormat(ty_uf);
	t[2].ToUnifiedFormat(tz_uf);

	q[0].ToUnifiedFormat(qw_uf);
	q[1].ToUnifiedFormat(qx_uf);
	q[2].ToUnifiedFormat(qy_uf);
	q[3].ToUnifiedFormat(qz_uf);

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
	PrepareWOut(result, n, otx, oty, otz, oqw, oqx, oqy, oqz);

	const bool all_valid =
	    t_uf.validity.CannotHaveNull() && q_uf.validity.CannotHaveNull() && tx_uf.validity.CannotHaveNull() &&
	    ty_uf.validity.CannotHaveNull() && tz_uf.validity.CannotHaveNull() && qw_uf.validity.CannotHaveNull() &&
	    qx_uf.validity.CannotHaveNull() && qy_uf.validity.CannotHaveNull() && qz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) ||
			    !RowIsValid(qz_uf, i)) {
				FlatVector::SetNull(result, i, true);
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

void Se3MakeFn(DataChunk &input, ExpressionState &, Vector &result) {
	MakeTransform(input.data[0], input.data[1], result, input.size());
}

// se3_from_axis_angle(t, axis, th) -> W
void Se3FromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t count = input.size();
	const bool constant_rotation = input.data[1].GetVectorType() == VectorType::CONSTANT_VECTOR &&
	                               input.data[2].GetVectorType() == VectorType::CONSTANT_VECTOR;
	if (CannotHaveNullComponents(input.data[0]) || constant_rotation) {
		Vector quaternion(StructVector::GetEntries(result)[1].GetType(), count);
		AxisAngleQuaternion(input.data[1], input.data[2], quaternion, count);
		MakeTransform(input.data[0], quaternion, result, count);
		return;
	}

	// Skip invalid translations before normalizing axes or evaluating trigonometric functions.
	auto translations = input.data[0].Values<Vec3Vector>();
	auto axes = input.data[1].Values<Vec3Vector>();
	auto angles = input.data[2].Values<double>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, count, tx, ty, tz, qw, qx, qy, qz);
	for (idx_t i = 0; i < count; i++) {
		Vec3 translation, axis;
		auto angle = angles[i];
		if (!ReadVec3<false>(translations[i], translation) || !ReadVec3<false>(axes[i], axis) || !angle.IsValid() ||
		    !AxisAngleToQuat(axis.x, axis.y, axis.z, angle.GetValue(), qw[i], qx[i], qy[i], qz[i])) {
			FlatVector::SetNull(result, i, true);
			continue;
		}
		tx[i] = translation.x;
		ty[i] = translation.y;
		tz[i] = translation.z;
	}
}

template <bool ALL_VALID>
static void Se3Apply(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	auto transforms = input.data[0].Values<TransformVector>();
	auto points = input.data[1].Values<Vec3Vector>();
	if constexpr (ALL_VALID) {
		double *x, *y, *z;
		PrepareVec3Out(result, n, x, y, z);
		for (idx_t i = 0; i < n; i++) {
			auto transform = transforms[i];
			Vec3 t, p;
			Quat q;
			ReadVec3<true>(transform.GetChildValue<0>(), t);
			ReadQuat<true>(transform.GetChildValue<1>(), q);
			ReadVec3<true>(points[i], p);
			QRotate(q.w, q.x, q.y, q.z, p.x + t.x, p.y + t.y, p.z + t.z, x[i], y[i], z[i]);
		}
	} else {
		auto writer = FlatVector::Writer<Vec3Vector>(result, n);
		for (idx_t i = 0; i < n; i++) {
			auto transform = transforms[i];
			Vec3 t, p;
			Quat q;
			if (!transform.IsValid() || !ReadVec3<false>(transform.GetChildValue<0>(), t) ||
			    !ReadQuat<false>(transform.GetChildValue<1>(), q) || !ReadVec3<false>(points[i], p)) {
				writer.WriteNull();
				continue;
			}
			WriteVec3(writer, Rotate(q, {p.x + t.x, p.y + t.y, p.z + t.z}));
		}
	}
}

void Se3ApplyFn(DataChunk &input, ExpressionState &, Vector &result) {
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		const idx_t count = input.size();
		const auto &transform = StructVector::GetEntries(input.data[0]);
		FlatVec3 translations(transform[0]), points(input.data[1]);
		FlatQuat rotations(transform[1]);
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			auto p = points[i];
			auto t = translations[i];
			auto q = rotations[i];
			QRotate(q.w, q.x, q.y, q.z, p.x + t.x, p.y + t.y, p.z + t.z, x[i], y[i], z[i]);
		}
		return;
	}
	if (CannotHaveNullComponents(input.data[0]) && CannotHaveNullComponents(input.data[1])) {
		Se3Apply<true>(input, result);
	} else {
		Se3Apply<false>(input, result);
	}
}

void Se3ApplyTFn(DataChunk &input, ExpressionState &, Vector &result) {
	AddVec3(input.data[1], input.data[0], result, input.size());
}

template <bool ALL_VALID>
static void Se3ApplyQ(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	auto rotations = input.data[0].Values<QuatVector>();
	auto points = input.data[1].Values<Vec3Vector>();
	if constexpr (ALL_VALID) {
		double *x, *y, *z;
		PrepareVec3Out(result, n, x, y, z);
		for (idx_t i = 0; i < n; i++) {
			Quat q;
			Vec3 p;
			ReadQuat<true>(rotations[i], q);
			ReadVec3<true>(points[i], p);
			QRotate(q.w, q.x, q.y, q.z, p.x, p.y, p.z, x[i], y[i], z[i]);
		}
	} else {
		auto writer = FlatVector::Writer<Vec3Vector>(result, n);
		for (idx_t i = 0; i < n; i++) {
			Quat q;
			Vec3 p;
			if (!ReadQuat<false>(rotations[i], q) || !ReadVec3<false>(points[i], p)) {
				writer.WriteNull();
				continue;
			}
			WriteVec3(writer, Rotate(q, p));
		}
	}
}

void Se3ApplyQFn(DataChunk &input, ExpressionState &, Vector &result) {
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		const idx_t count = input.size();
		FlatQuat rotations(input.data[0]);
		FlatVec3 points(input.data[1]);
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			auto q = rotations[i];
			auto p = points[i];
			QRotate(q.w, q.x, q.y, q.z, p.x, p.y, p.z, x[i], y[i], z[i]);
		}
		return;
	}
	if (input.data[0].GetVectorType() == VectorType::CONSTANT_VECTOR && CannotHaveNullComponents(input.data[0]) &&
	    IsFlatAndValid(input.data[1])) {
		const idx_t count = input.size();
		auto rotations = input.data[0].Values<QuatVector>();
		Quat q;
		ReadQuat<true>(rotations[0], q);
		FlatVec3 points(input.data[1]);
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			auto p = points[i];
			QRotate(q.w, q.x, q.y, q.z, p.x, p.y, p.z, x[i], y[i], z[i]);
		}
		return;
	}
	if (CannotHaveNullComponents(input.data[0]) && CannotHaveNullComponents(input.data[1])) {
		Se3ApplyQ<true>(input, result);
	} else {
		Se3ApplyQ<false>(input, result);
	}
}

// se3_inv(t) -> t (negation)
void Se3InvTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryNegateComponents(input.data[0], result, n, 0)) {
		return;
	}

	auto &t_v = input.data[0];
	auto &t = StructVector::GetEntries(t_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, t_uf;
	t_v.ToUnifiedFormat(t_uf);
	t[0].ToUnifiedFormat(tx_uf);
	t[1].ToUnifiedFormat(ty_uf);
	t[2].ToUnifiedFormat(tz_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	const bool all_valid = t_uf.validity.CannotHaveNull() && tx_uf.validity.CannotHaveNull() &&
	                       ty_uf.validity.CannotHaveNull() && tz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i)) {
				FlatVector::SetNull(result, i, true);
				continue;
			}
		}
		ox[i] = -tx[tx_uf.sel->get_index(i)];
		oy[i] = -ty[ty_uf.sel->get_index(i)];
		oz[i] = -tz[tz_uf.sel->get_index(i)];
	}
}

template <bool ALL_VALID>
static void Se3InvW(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	auto transforms = input.data[0].Values<TransformVector>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	for (idx_t i = 0; i < n; i++) {
		Vec3 t;
		Quat q;
		if (!ReadTransform<ALL_VALID>(transforms[i], t, q)) {
			FlatVector::SetNull(result, i, true);
			continue;
		}
		double rx, ry, rz;
		QRotate(q.w, q.x, q.y, q.z, t.x, t.y, t.z, rx, ry, rz);
		tx[i] = -rx;
		ty[i] = -ry;
		tz[i] = -rz;
		qw[i] = q.w;
		qx[i] = -q.x;
		qy[i] = -q.y;
		qz[i] = -q.z;
	}
}

// se3_inv(W) -> W such that W_inv(W(p)) = p
void Se3InvWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (IsFlatAndValid(input.data[0])) {
		const auto &children = StructVector::GetEntries(input.data[0]);
		FlatVec3 translations(children[0]);
		FlatQuat rotations(children[1]);
		double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
		PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
		for (idx_t i = 0; i < n; i++) {
			auto q = rotations[i];
			auto t = Rotate(q, translations[i]);
			tx[i] = -t.x;
			ty[i] = -t.y;
			tz[i] = -t.z;
			qw[i] = q.w;
			qx[i] = -q.x;
			qy[i] = -q.y;
			qz[i] = -q.z;
		}
		return;
	}

	if (CannotHaveNullComponents(input.data[0])) {
		Se3InvW<true>(input, result);
	} else {
		Se3InvW<false>(input, result);
	}
}

// se3_compose overloads: result equals applying B, then A
// W = A ∘ B  with convention W(p)=R_q(p+t)

template <bool ALL_VALID>
static void Se3ComposeWW(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	auto left = input.data[0].Values<TransformVector>();
	auto right = input.data[1].Values<TransformVector>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	for (idx_t i = 0; i < n; i++) {
		Vec3 at, bt;
		Quat aq, bq;
		if (!ReadTransform<ALL_VALID>(left[i], at, aq) || !ReadTransform<ALL_VALID>(right[i], bt, bq)) {
			FlatVector::SetNull(result, i, true);
			continue;
		}
		double rx, ry, rz;
		QInvRotate(bq.w, bq.x, bq.y, bq.z, at.x, at.y, at.z, rx, ry, rz);
		tx[i] = bt.x + rx;
		ty[i] = bt.y + ry;
		tz[i] = bt.z + rz;
		double rw, rqx, rqy, rqz;
		QMul(aq.w, aq.x, aq.y, aq.z, bq.w, bq.x, bq.y, bq.z, rw, rqx, rqy, rqz);
		qw[i] = rw;
		qx[i] = rqx;
		qy[i] = rqy;
		qz[i] = rqz;
	}
}

// se3_compose(WA, WB) -> W
void Se3ComposeWWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		const auto &a = StructVector::GetEntries(input.data[0]);
		const auto &b = StructVector::GetEntries(input.data[1]);
		FlatVec3 a_translations(a[0]), b_translations(b[0]);
		FlatQuat a_rotations(a[1]), b_rotations(b[1]);
		PrepareStructChildrenFlat(result, n);
		auto &output = StructVector::GetEntries(result);
		VectorWriter<Vec3Vector> t_writer(output[0], n, 0);
		VectorWriter<QuatVector> q_writer(output[1], n, 0);
		for (idx_t i = 0; i < n; i++) {
			auto at = a_translations[i];
			auto bq = b_rotations[i];
			Vec3 rotated;
			QInvRotate(bq.w, bq.x, bq.y, bq.z, at.x, at.y, at.z, rotated.x, rotated.y, rotated.z);
			auto bt = b_translations[i];
			WriteVec3(t_writer, {bt.x + rotated.x, bt.y + rotated.y, bt.z + rotated.z});
			auto aq = a_rotations[i];
			Quat q;
			QMul(aq.w, aq.x, aq.y, aq.z, bq.w, bq.x, bq.y, bq.z, q.w, q.x, q.y, q.z);
			WriteQuat(q_writer, q);
		}
		return;
	}

	if (CannotHaveNullComponents(input.data[0]) && CannotHaveNullComponents(input.data[1])) {
		Se3ComposeWW<true>(input, result);
	} else {
		Se3ComposeWW<false>(input, result);
	}
}

// se3_compose(vec3, vec3) -> vec3 (translation add)
void Se3ComposeTTFn(DataChunk &input, ExpressionState &, Vector &result) {
	AddVec3(input.data[0], input.data[1], result, input.size());
}

// se3_compose(q, t) -> W (apply t then q)
void Se3ComposeQTFn(DataChunk &input, ExpressionState &, Vector &result) {
	MakeTransform(input.data[1], input.data[0], result, input.size());
}

// se3_compose(t, q) -> W (apply q then t)
void Se3ComposeTQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &q_v = input.data[1];
	auto &t = StructVector::GetEntries(t_v);
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf, tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat q_uf, t_uf;
	q_v.ToUnifiedFormat(q_uf);
	t_v.ToUnifiedFormat(t_uf);
	q[0].ToUnifiedFormat(qw_uf);
	q[1].ToUnifiedFormat(qx_uf);
	q[2].ToUnifiedFormat(qy_uf);
	q[3].ToUnifiedFormat(qz_uf);
	t[0].ToUnifiedFormat(tx_uf);
	t[1].ToUnifiedFormat(ty_uf);
	t[2].ToUnifiedFormat(tz_uf);

	auto *qw = (const double *)qw_uf.data;
	auto *qx = (const double *)qx_uf.data;
	auto *qy = (const double *)qy_uf.data;
	auto *qz = (const double *)qz_uf.data;
	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;

	double *otx, *oty, *otz, *oqw, *oqx, *oqy, *oqz;
	PrepareWOut(result, n, otx, oty, otz, oqw, oqx, oqy, oqz);

	const bool all_valid =
	    q_uf.validity.CannotHaveNull() && t_uf.validity.CannotHaveNull() && qw_uf.validity.CannotHaveNull() &&
	    qx_uf.validity.CannotHaveNull() && qy_uf.validity.CannotHaveNull() && qz_uf.validity.CannotHaveNull() &&
	    tx_uf.validity.CannotHaveNull() && ty_uf.validity.CannotHaveNull() && tz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) ||
			    !RowIsValid(tz_uf, i)) {
				FlatVector::SetNull(result, i, true);
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
void Se3ComposeWQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &w_v = input.data[0];
	auto &q_v = input.data[1];
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(went[0]);
	auto &qent = StructVector::GetEntries(went[1]);
	auto &q2ent = StructVector::GetEntries(q_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat q2w_uf, q2x_uf, q2y_uf, q2z_uf;
	UnifiedVectorFormat w_uf, t_uf, q_uf, q2_uf;

	w_v.ToUnifiedFormat(w_uf);
	went[0].ToUnifiedFormat(t_uf);
	went[1].ToUnifiedFormat(q_uf);
	q_v.ToUnifiedFormat(q2_uf);

	tent[0].ToUnifiedFormat(tx_uf);
	tent[1].ToUnifiedFormat(ty_uf);
	tent[2].ToUnifiedFormat(tz_uf);
	qent[0].ToUnifiedFormat(qw_uf);
	qent[1].ToUnifiedFormat(qx_uf);
	qent[2].ToUnifiedFormat(qy_uf);
	qent[3].ToUnifiedFormat(qz_uf);

	q2ent[0].ToUnifiedFormat(q2w_uf);
	q2ent[1].ToUnifiedFormat(q2x_uf);
	q2ent[2].ToUnifiedFormat(q2y_uf);
	q2ent[3].ToUnifiedFormat(q2z_uf);

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
	PrepareWOut(result, n, otx, oty, otz, oqw, oqx, oqy, oqz);

	const bool all_valid =
	    w_uf.validity.CannotHaveNull() && t_uf.validity.CannotHaveNull() && q_uf.validity.CannotHaveNull() &&
	    q2_uf.validity.CannotHaveNull() && tx_uf.validity.CannotHaveNull() && ty_uf.validity.CannotHaveNull() &&
	    tz_uf.validity.CannotHaveNull() && qw_uf.validity.CannotHaveNull() && qx_uf.validity.CannotHaveNull() &&
	    qy_uf.validity.CannotHaveNull() && qz_uf.validity.CannotHaveNull() && q2w_uf.validity.CannotHaveNull() &&
	    q2x_uf.validity.CannotHaveNull() && q2y_uf.validity.CannotHaveNull() && q2z_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(q2_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) ||
			    !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(q2w_uf, i) ||
			    !RowIsValid(q2x_uf, i) || !RowIsValid(q2y_uf, i) || !RowIsValid(q2z_uf, i)) {
				FlatVector::SetNull(result, i, true);
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
void Se3ComposeQWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (CannotHaveNullComponents(input.data[0]) && CannotHaveNullComponents(input.data[1])) {
		const auto &transform = StructVector::GetEntries(input.data[1]);
		PrepareStructChildrenFlat(result, n);
		auto &output = StructVector::GetEntries(result);
		output[0].Reference(transform[0]);
		MultiplyQuaternions(input.data[0], transform[1], output[1], n);
		return;
	}

	auto &q_v = input.data[0];
	auto &w_v = input.data[1];
	auto &qent = StructVector::GetEntries(q_v);
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(went[0]);
	auto &wqent = StructVector::GetEntries(went[1]);

	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, wqw_uf, wqx_uf, wqy_uf, wqz_uf;
	UnifiedVectorFormat q_uf, w_uf, t_uf, wq_uf;

	q_v.ToUnifiedFormat(q_uf);
	w_v.ToUnifiedFormat(w_uf);
	went[0].ToUnifiedFormat(t_uf);
	went[1].ToUnifiedFormat(wq_uf);

	qent[0].ToUnifiedFormat(qw_uf);
	qent[1].ToUnifiedFormat(qx_uf);
	qent[2].ToUnifiedFormat(qy_uf);
	qent[3].ToUnifiedFormat(qz_uf);
	tent[0].ToUnifiedFormat(tx_uf);
	tent[1].ToUnifiedFormat(ty_uf);
	tent[2].ToUnifiedFormat(tz_uf);
	wqent[0].ToUnifiedFormat(wqw_uf);
	wqent[1].ToUnifiedFormat(wqx_uf);
	wqent[2].ToUnifiedFormat(wqy_uf);
	wqent[3].ToUnifiedFormat(wqz_uf);

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
	PrepareWOut(result, n, otx, oty, otz, oqw, oqx, oqy, oqz);

	const bool all_valid =
	    q_uf.validity.CannotHaveNull() && w_uf.validity.CannotHaveNull() && t_uf.validity.CannotHaveNull() &&
	    wq_uf.validity.CannotHaveNull() && qw_uf.validity.CannotHaveNull() && qx_uf.validity.CannotHaveNull() &&
	    qy_uf.validity.CannotHaveNull() && qz_uf.validity.CannotHaveNull() && tx_uf.validity.CannotHaveNull() &&
	    ty_uf.validity.CannotHaveNull() && tz_uf.validity.CannotHaveNull() && wqw_uf.validity.CannotHaveNull() &&
	    wqx_uf.validity.CannotHaveNull() && wqy_uf.validity.CannotHaveNull() && wqz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(q_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(wq_uf, i) ||
			    !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(wqw_uf, i) ||
			    !RowIsValid(wqx_uf, i) || !RowIsValid(wqy_uf, i) || !RowIsValid(wqz_uf, i)) {
				FlatVector::SetNull(result, i, true);
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
void Se3ComposeWTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (CannotHaveNullComponents(input.data[0]) && CannotHaveNullComponents(input.data[1])) {
		const auto &transform = StructVector::GetEntries(input.data[0]);
		PrepareStructChildrenFlat(result, n);
		auto &output = StructVector::GetEntries(result);
		AddVec3(transform[0], input.data[1], output[0], n);
		output[1].Reference(transform[1]);
		return;
	}

	auto &w_v = input.data[0];
	auto &t_v = input.data[1];
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(went[0]);
	auto &qent = StructVector::GetEntries(went[1]);
	auto &t2 = StructVector::GetEntries(t_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat t2x_uf, t2y_uf, t2z_uf;
	UnifiedVectorFormat w_uf, t_uf, q_uf, t2_uf;

	w_v.ToUnifiedFormat(w_uf);
	went[0].ToUnifiedFormat(t_uf);
	went[1].ToUnifiedFormat(q_uf);
	t_v.ToUnifiedFormat(t2_uf);

	tent[0].ToUnifiedFormat(tx_uf);
	tent[1].ToUnifiedFormat(ty_uf);
	tent[2].ToUnifiedFormat(tz_uf);
	qent[0].ToUnifiedFormat(qw_uf);
	qent[1].ToUnifiedFormat(qx_uf);
	qent[2].ToUnifiedFormat(qy_uf);
	qent[3].ToUnifiedFormat(qz_uf);

	t2[0].ToUnifiedFormat(t2x_uf);
	t2[1].ToUnifiedFormat(t2y_uf);
	t2[2].ToUnifiedFormat(t2z_uf);

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
	PrepareWOut(result, n, otx, oty, otz, oqw, oqx, oqy, oqz);

	const bool all_valid =
	    w_uf.validity.CannotHaveNull() && t_uf.validity.CannotHaveNull() && q_uf.validity.CannotHaveNull() &&
	    t2_uf.validity.CannotHaveNull() && tx_uf.validity.CannotHaveNull() && ty_uf.validity.CannotHaveNull() &&
	    tz_uf.validity.CannotHaveNull() && qw_uf.validity.CannotHaveNull() && qx_uf.validity.CannotHaveNull() &&
	    qy_uf.validity.CannotHaveNull() && qz_uf.validity.CannotHaveNull() && t2x_uf.validity.CannotHaveNull() &&
	    t2y_uf.validity.CannotHaveNull() && t2z_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) || !RowIsValid(t2_uf, i) ||
			    !RowIsValid(tx_uf, i) || !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) ||
			    !RowIsValid(qx_uf, i) || !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i) || !RowIsValid(t2x_uf, i) ||
			    !RowIsValid(t2y_uf, i) || !RowIsValid(t2z_uf, i)) {
				FlatVector::SetNull(result, i, true);
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
void Se3ComposeTWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();

	auto &t_v = input.data[0];
	auto &w_v = input.data[1];
	auto &t2 = StructVector::GetEntries(t_v);
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(went[0]);
	auto &qent = StructVector::GetEntries(went[1]);

	UnifiedVectorFormat t2x_uf, t2y_uf, t2z_uf;
	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;
	UnifiedVectorFormat t2_uf, w_uf, t_uf, q_uf;

	t_v.ToUnifiedFormat(t2_uf);
	w_v.ToUnifiedFormat(w_uf);
	went[0].ToUnifiedFormat(t_uf);
	went[1].ToUnifiedFormat(q_uf);

	t2[0].ToUnifiedFormat(t2x_uf);
	t2[1].ToUnifiedFormat(t2y_uf);
	t2[2].ToUnifiedFormat(t2z_uf);

	tent[0].ToUnifiedFormat(tx_uf);
	tent[1].ToUnifiedFormat(ty_uf);
	tent[2].ToUnifiedFormat(tz_uf);
	qent[0].ToUnifiedFormat(qw_uf);
	qent[1].ToUnifiedFormat(qx_uf);
	qent[2].ToUnifiedFormat(qy_uf);
	qent[3].ToUnifiedFormat(qz_uf);

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
	PrepareWOut(result, n, otx, oty, otz, oqw, oqx, oqy, oqz);

	const bool all_valid =
	    t2_uf.validity.CannotHaveNull() && w_uf.validity.CannotHaveNull() && t_uf.validity.CannotHaveNull() &&
	    q_uf.validity.CannotHaveNull() && t2x_uf.validity.CannotHaveNull() && t2y_uf.validity.CannotHaveNull() &&
	    t2z_uf.validity.CannotHaveNull() && tx_uf.validity.CannotHaveNull() && ty_uf.validity.CannotHaveNull() &&
	    tz_uf.validity.CannotHaveNull() && qw_uf.validity.CannotHaveNull() && qx_uf.validity.CannotHaveNull() &&
	    qy_uf.validity.CannotHaveNull() && qz_uf.validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid) {
			if (!RowIsValid(t2_uf, i) || !RowIsValid(w_uf, i) || !RowIsValid(t_uf, i) || !RowIsValid(q_uf, i) ||
			    !RowIsValid(t2x_uf, i) || !RowIsValid(t2y_uf, i) || !RowIsValid(t2z_uf, i) || !RowIsValid(tx_uf, i) ||
			    !RowIsValid(ty_uf, i) || !RowIsValid(tz_uf, i) || !RowIsValid(qw_uf, i) || !RowIsValid(qx_uf, i) ||
			    !RowIsValid(qy_uf, i) || !RowIsValid(qz_uf, i)) {
				FlatVector::SetNull(result, i, true);
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

} // namespace se3
} // namespace duckdb
