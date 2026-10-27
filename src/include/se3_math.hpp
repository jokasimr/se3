#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace duckdb {
namespace se3 {

struct Vec3 {
	double x, y, z;
};

struct Quat {
	double w, x, y, z;
};

// ------------------------- Quaternion kernels -------------------------
static inline void QMul(double aw, double ax, double ay, double az, double bw, double bx, double by, double bz,
                        double &rw, double &rx, double &ry, double &rz) {
	rw = aw * bw - ax * bx - ay * by - az * bz;
	rx = aw * bx + ax * bw + ay * bz - az * by;
	ry = aw * by - ax * bz + ay * bw + az * bx;
	rz = aw * bz + ax * by - ay * bx + az * bw;
}

static inline double MaxAbs3(double x, double y, double z);
static inline bool AllFinite3(double x, double y, double z);
static inline bool AllFinite4(double w, double x, double y, double z);

static inline void QRotateFast(double qw, double qx, double qy, double qz, double vx, double vy, double vz, double &ox,
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

static inline void QRotate(double qw, double qx, double qy, double qz, double vx, double vy, double vz, double &ox,
                           double &oy, double &oz) {
	QRotateFast(qw, qx, qy, qz, vx, vy, vz, ox, oy, oz);
	if (std::isfinite(ox) && std::isfinite(oy) && std::isfinite(oz)) {
		return;
	}
	if (!AllFinite4(qw, qx, qy, qz) || !AllFinite3(vx, vy, vz)) {
		return;
	}

	const double scale = MaxAbs3(vx, vy, vz);
	if (scale == 0.0) {
		return;
	}

	double sx, sy, sz;
	QRotateFast(qw, qx, qy, qz, vx / scale, vy / scale, vz / scale, sx, sy, sz);
	ox = sx * scale;
	oy = sy * scale;
	oz = sz * scale;
}

static inline void QInvRotate(double qw, double qx, double qy, double qz, double vx, double vy, double vz, double &ox,
                              double &oy, double &oz) {
	QRotate(qw, -qx, -qy, -qz, vx, vy, vz, ox, oy, oz);
}

static inline double MaxAbs3(double x, double y, double z) {
	const double ax = std::fabs(x);
	const double ay = std::fabs(y);
	const double az = std::fabs(z);
	const double m = ax > ay ? ax : ay;
	return m > az ? m : az;
}

static inline double MaxAbs4(double w, double x, double y, double z) {
	const double aw_abs = std::fabs(w);
	const double ax = std::fabs(x);
	const double ay = std::fabs(y);
	const double az = std::fabs(z);
	const double m0 = aw_abs > ax ? aw_abs : ax;
	const double m1 = ay > az ? ay : az;
	return m0 > m1 ? m0 : m1;
}

static constexpr double SQRT_DBL_MIN = 1.4916681462400413e-154;
static constexpr double DBL_MIN_NORMAL = std::numeric_limits<double>::min();

static inline bool AllFinite3(double x, double y, double z) {
	return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

static inline bool AllFinite4(double w, double x, double y, double z) {
	return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

enum class AngleFunction { COSINE, SINE, TANGENT, COTANGENT, ANGLE };

template <std::size_t N>
static inline double AngleDot(const std::array<double, N> &a, const std::array<double, N> &b) {
	// Starting with a product rather than +0 preserves an all-negative-zero sum.
	double dot = a[0] * b[0];
	for (std::size_t i = 1; i < N; i++) {
		dot += a[i] * b[i];
	}
	return dot;
}

template <std::size_t N>
static inline std::array<double, N *(N - 1) / 2> AngleMinors(const std::array<double, N> &a,
                                                             const std::array<double, N> &b) {
	std::array<double, N *(N - 1) / 2> minors;
	std::size_t index = 0;
	for (std::size_t i = 0; i < N; i++) {
		for (std::size_t j = i + 1; j < N; j++) {
			// Kahan's difference of products: recover the rounding error in the
			// second product before adding it to the fused subtraction.
			// https://perso.ens-lyon.fr/jean-michel.muller/AnalysisCornea_final.pdf
			const double product = a[j] * b[i];
			const double error = std::fma(-a[j], b[i], product);
			minors[index++] = std::fma(a[i], b[j], -product) + error;
		}
	}
	return minors;
}

template <std::size_t N>
static double AngleNorm(const std::array<double, N> &values) {
	// Only used for minors of scaled vectors: finite values cannot overflow
	// when squared, but a small area can still underflow.
	const double norm2 = AngleDot(values, values);
	if (norm2 >= DBL_MIN_NORMAL || !std::isfinite(norm2)) {
		return std::sqrt(norm2);
	}
	// Squaring a tiny, representable component can underflow.
	double scale = 0.0;
	for (const auto value : values) {
		scale = std::max(scale, std::fabs(value));
	}
	if (scale == 0.0) {
		return 0.0;
	}
	double scaled_norm2 = 0.0;
	for (const auto value : values) {
		const double scaled = value / scale;
		scaled_norm2 += scaled * scaled;
	}
	return scale * std::sqrt(scaled_norm2);
}

template <AngleFunction FUNCTION>
static inline double AngleFromGeometry(double area, double dot, double norm_product) {
	if constexpr (FUNCTION == AngleFunction::COSINE) {
		return dot / std::sqrt(norm_product);
	} else if constexpr (FUNCTION == AngleFunction::SINE) {
		return area / std::sqrt(norm_product);
	} else if constexpr (FUNCTION == AngleFunction::TANGENT) {
		return area / dot;
	} else if constexpr (FUNCTION == AngleFunction::COTANGENT) {
		return dot / area;
	} else {
		return std::atan2(area, dot);
	}
}

template <std::size_t N, AngleFunction FUNCTION>
static inline double VectorAngle(const std::array<double, N> &a, const std::array<double, N> &b) {
	if constexpr (FUNCTION == AngleFunction::COSINE) {
		const double dot = AngleDot(a, b);
		const double na2 = AngleDot(a, a), nb2 = AngleDot(b, b);
		const double denom = std::sqrt(na2 * nb2);
		if (na2 >= DBL_MIN_NORMAL && nb2 >= DBL_MIN_NORMAL && denom >= SQRT_DBL_MIN && std::isfinite(denom) &&
		    std::isfinite(dot)) {
			return dot / denom;
		}
	} else {
		const auto minors = AngleMinors(a, b);
		const double area2 = AngleDot(minors, minors);
		if (area2 >= DBL_MIN_NORMAL && std::isfinite(area2)) {
			if constexpr (FUNCTION == AngleFunction::SINE) {
				const double na2 = AngleDot(a, a), nb2 = AngleDot(b, b);
				const double norm_product = na2 * nb2;
				const double sine2 = area2 / norm_product;
				if (na2 >= DBL_MIN_NORMAL && nb2 >= DBL_MIN_NORMAL && norm_product >= DBL_MIN_NORMAL &&
				    std::isfinite(norm_product) && sine2 >= DBL_MIN_NORMAL) {
					// One square root suffices while the squared ratio is normal.
					return std::sqrt(sine2);
				}
			} else {
				const double dot = AngleDot(a, b);
				if (std::isfinite(dot)) {
					return AngleFromGeometry<FUNCTION>(std::sqrt(area2), dot, 0.0);
				}
			}
		}
	}

	// Rescale each vector when intermediate magnitudes are unsafe.
	// Zero or non-finite vectors naturally propagate NaN through this
	// arithmetic. Dot products retain ordinary double rounding/cancellation.
	double ascale = 0.0, bscale = 0.0;
	for (std::size_t i = 0; i < N; i++) {
		ascale = std::max(ascale, std::fabs(a[i]));
		bscale = std::max(bscale, std::fabs(b[i]));
	}
	std::array<double, N> sa, sb;
	for (std::size_t i = 0; i < N; i++) {
		sa[i] = a[i] / ascale;
		sb[i] = b[i] / bscale;
	}
	double area = 0.0, dot = 0.0, norm_product = 0.0;
	if constexpr (FUNCTION != AngleFunction::COSINE) {
		area = AngleNorm(AngleMinors(sa, sb));
	}
	if constexpr (FUNCTION != AngleFunction::SINE) {
		dot = AngleDot(sa, sb);
	}
	if constexpr (FUNCTION == AngleFunction::COSINE || FUNCTION == AngleFunction::SINE) {
		norm_product = AngleDot(sa, sa) * AngleDot(sb, sb);
	}
	return AngleFromGeometry<FUNCTION>(area, dot, norm_product);
}

static inline double ScaledNorm3(double x, double y, double z, double scale) {
	const double sx = x / scale;
	const double sy = y / scale;
	const double sz = z / scale;
	return scale * std::sqrt(sx * sx + sy * sy + sz * sz);
}

static inline double ScaledNorm4(double w, double x, double y, double z, double scale) {
	const double sw = w / scale;
	const double sx = x / scale;
	const double sy = y / scale;
	const double sz = z / scale;
	return scale * std::sqrt(sw * sw + sx * sx + sy * sy + sz * sz);
}

static inline double Norm3(double x, double y, double z) {
	const double n2 = x * x + y * y + z * z;
	if (n2 >= DBL_MIN_NORMAL && std::isfinite(n2)) {
		return std::sqrt(n2);
	}
	if (AllFinite3(x, y, z)) {
		const double scale = MaxAbs3(x, y, z);
		if (scale != 0.0) {
			return ScaledNorm3(x, y, z, scale);
		}
	}
	return std::sqrt(n2);
}

static inline double Norm4(double w, double x, double y, double z) {
	const double n2 = w * w + x * x + y * y + z * z;
	if (n2 >= DBL_MIN_NORMAL && std::isfinite(n2)) {
		return std::sqrt(n2);
	}
	if (AllFinite4(w, x, y, z)) {
		const double scale = MaxAbs4(w, x, y, z);
		if (scale != 0.0) {
			return ScaledNorm4(w, x, y, z, scale);
		}
	}
	return std::sqrt(n2);
}

static inline void Normalize3(double x, double y, double z, double &ox, double &oy, double &oz) {
	const double n2 = x * x + y * y + z * z;
	if (n2 >= DBL_MIN_NORMAL && std::isfinite(n2)) {
		const double inv_n = 1.0 / std::sqrt(n2);
		ox = x * inv_n;
		oy = y * inv_n;
		oz = z * inv_n;
		return;
	}
	if (AllFinite3(x, y, z)) {
		const double scale = MaxAbs3(x, y, z);
		if (scale != 0.0) {
			const double sx = x / scale;
			const double sy = y / scale;
			const double sz = z / scale;
			const double scaled_n = std::sqrt(sx * sx + sy * sy + sz * sz);
			ox = sx / scaled_n;
			oy = sy / scaled_n;
			oz = sz / scaled_n;
			return;
		}
	}

	const double inv_n = 1.0 / std::sqrt(n2);
	ox = x * inv_n;
	oy = y * inv_n;
	oz = z * inv_n;
}

static inline void Normalize4(double w, double x, double y, double z, double &ow, double &ox, double &oy, double &oz) {
	const double n2 = w * w + x * x + y * y + z * z;
	if (n2 >= DBL_MIN_NORMAL && std::isfinite(n2)) {
		const double inv_n = 1.0 / std::sqrt(n2);
		ow = w * inv_n;
		ox = x * inv_n;
		oy = y * inv_n;
		oz = z * inv_n;
		return;
	}
	if (AllFinite4(w, x, y, z)) {
		const double scale = MaxAbs4(w, x, y, z);
		if (scale != 0.0) {
			const double sw = w / scale;
			const double sx = x / scale;
			const double sy = y / scale;
			const double sz = z / scale;
			const double scaled_n = std::sqrt(sw * sw + sx * sx + sy * sy + sz * sz);
			ow = sw / scaled_n;
			ox = sx / scaled_n;
			oy = sy / scaled_n;
			oz = sz / scaled_n;
			return;
		}
	}

	const double inv_n = 1.0 / std::sqrt(n2);
	ow = w * inv_n;
	ox = x * inv_n;
	oy = y * inv_n;
	oz = z * inv_n;
}

static inline bool AxisAngleToQuat(double ax, double ay, double az, double th, double &ow, double &ox, double &oy,
                                   double &oz) {
	if (ax == 0.0 && ay == 0.0 && az == 0.0) {
		return false;
	}

	double ux, uy, uz;
	Normalize3(ax, ay, az, ux, uy, uz);

	const double h = 0.5 * th;
	const double s = std::sin(h);
	ow = std::cos(h);
	ox = ux * s;
	oy = uy * s;
	oz = uz * s;
	return true;
}

static inline bool ProductCannotUnderflow(double a, double b) {
	return (std::abs(a) >= SQRT_DBL_MIN && std::abs(b) >= SQRT_DBL_MIN) || a == 0.0 || b == 0.0 ||
	       std::abs(a) > DBL_MIN_NORMAL / std::abs(b);
}

template <std::size_t N, bool REJECT>
static std::array<double, N> WideProjection(const std::array<double, N> &a, const std::array<double, N> &b,
                                            double bscale) {
	static_assert(N == 3 || N == 4, "Projection kernels support three or four dimensions");
	int bexp;
	std::frexp(bscale, &bexp);
	struct Product {
		int exponent;
		double fraction;
	};
	std::array<Product, N> products;
	std::array<double, N> bfractions;
	std::array<int, N> bexponents;
	double bn2 = 0.0;
	for (std::size_t i = 0; i < N; i++) {
		int aexp;
		const double afraction = std::frexp(a[i], &aexp);
		bfractions[i] = std::frexp(b[i], &bexponents[i]);
		products[i] = {aexp + bexponents[i], afraction * bfractions[i]};
		const double scaled_b = std::scalbn(b[i], -bexp);
		bn2 += scaled_b * scaled_b;
	}
	std::sort(products.begin(), products.end(),
	          [](const Product &x, const Product &y) { return x.exponent > y.exponent; });

	double dot = 0.0;
	int dot_exp = 0;
	for (const auto &product : products) {
		if (product.fraction == 0.0) {
			continue;
		}
		if (dot == 0.0) {
			dot = product.fraction;
			dot_exp = product.exponent;
		} else {
			dot += std::scalbn(product.fraction, product.exponent - dot_exp);
		}
	}

	const double coefficient = dot / bn2;
	std::array<double, N> result;
	for (std::size_t i = 0; i < N; i++) {
		const double fraction = coefficient * bfractions[i];
		const int exponent = dot_exp + bexponents[i] - 2 * bexp;
		const double projected = std::scalbn(fraction, exponent);
		if (REJECT) {
			if (std::isfinite(projected)) {
				result[i] = a[i] - projected;
			} else {
				// In 3D/4D, half the projection fits even when the full projection overflows.
				result[i] = (a[i] * 0.5 - std::scalbn(fraction, exponent - 1)) * 2.0;
			}
		} else {
			result[i] = projected;
		}
	}
	return result;
}

template <std::size_t N, bool REJECT>
static std::array<double, N> ScaledProjection(const std::array<double, N> &a, const std::array<double, N> &b,
                                              double bscale) {
	static_assert(N == 3 || N == 4, "Projection kernels support three or four dimensions");
	std::array<double, N> scaled_a, scaled_b;
	const double minimum_b = bscale > 1.0 ? bscale * DBL_MIN_NORMAL : 0.0;
	double dot = 0.0, bn2 = 0.0;
	for (std::size_t i = 0; i < N; i++) {
		if ((a[i] != 0.0 && std::abs(a[i]) < 4.0 * DBL_MIN_NORMAL) || (b[i] != 0.0 && std::abs(b[i]) < minimum_b)) {
			return WideProjection<N, REJECT>(a, b, bscale);
		}
		// A fixed power of two bounds the sum without normalizing a by its largest component.
		scaled_a[i] = a[i] * 0.25;
		scaled_b[i] = b[i] / bscale;
		if (b[i] != 0.0 && !std::isnormal(scaled_b[i])) {
			return WideProjection<N, REJECT>(a, b, bscale);
		}
		dot += scaled_a[i] * scaled_b[i];
		bn2 += scaled_b[i] * scaled_b[i];
	}
	const double coefficient = dot / bn2;
	if (dot == 0.0) {
		for (std::size_t i = 0; i < N; i++) {
			if (!ProductCannotUnderflow(scaled_a[i], scaled_b[i])) {
				return WideProjection<N, REJECT>(a, b, bscale);
			}
		}
	} else if (!std::isnormal(dot) || !std::isnormal(coefficient)) {
		return WideProjection<N, REJECT>(a, b, bscale);
	}
	std::array<double, N> result;
	for (std::size_t i = 0; i < N; i++) {
		const double projected = (4.0 * scaled_b[i]) * coefficient;
		if (REJECT) {
			result[i] = std::isfinite(projected) ? a[i] - projected : (scaled_a[i] - scaled_b[i] * coefficient) * 4.0;
		} else {
			result[i] = projected;
		}
	}
	return result;
}

template <bool REJECT>
static inline double ProjectionComponent(double a, double b, double coefficient) {
	return REJECT ? a - b * coefficient : b * coefficient;
}

template <bool REJECT = false>
static inline void Projection3(double ax, double ay, double az, double bx, double by, double bz, double &ox, double &oy,
                               double &oz) {
	const double bn2 = bx * bx + by * by + bz * bz;
	const double dot = ax * bx + ay * by + az * bz;
	const double s = dot / bn2;
	ox = ProjectionComponent<REJECT>(ax, bx, s);
	oy = ProjectionComponent<REJECT>(ay, by, s);
	oz = ProjectionComponent<REJECT>(az, bz, s);

	if (bn2 >= DBL_MIN_NORMAL && std::isfinite(bn2) &&
	    ((std::isnormal(dot) && std::isnormal(s)) ||
	     (dot == 0.0 && ProductCannotUnderflow(ax, bx) && ProductCannotUnderflow(ay, by) &&
	      ProductCannotUnderflow(az, bz)))) {
		return;
	}
	if (!AllFinite3(ax, ay, az) || !AllFinite3(bx, by, bz)) {
		return;
	}

	const double bscale = MaxAbs3(bx, by, bz);
	if (bscale == 0.0) {
		return;
	}
	if (ax == 0.0 && ay == 0.0 && az == 0.0) {
		ox = oy = oz = 0.0;
		return;
	}
	const auto projected = ScaledProjection<3, REJECT>({ax, ay, az}, {bx, by, bz}, bscale);
	ox = projected[0];
	oy = projected[1];
	oz = projected[2];
}

template <bool REJECT = false>
static inline void Projection4(double aw, double ax, double ay, double az, double bw, double bx, double by, double bz,
                               double &ow, double &ox, double &oy, double &oz) {
	const double bn2 = bw * bw + bx * bx + by * by + bz * bz;
	const double dot = aw * bw + ax * bx + ay * by + az * bz;
	const double s = dot / bn2;
	ow = ProjectionComponent<REJECT>(aw, bw, s);
	ox = ProjectionComponent<REJECT>(ax, bx, s);
	oy = ProjectionComponent<REJECT>(ay, by, s);
	oz = ProjectionComponent<REJECT>(az, bz, s);

	if (bn2 >= DBL_MIN_NORMAL && std::isfinite(bn2) &&
	    ((std::isnormal(dot) && std::isnormal(s)) ||
	     (dot == 0.0 && ProductCannotUnderflow(aw, bw) && ProductCannotUnderflow(ax, bx) &&
	      ProductCannotUnderflow(ay, by) && ProductCannotUnderflow(az, bz)))) {
		return;
	}
	if (!AllFinite4(aw, ax, ay, az) || !AllFinite4(bw, bx, by, bz)) {
		return;
	}

	const double bscale = MaxAbs4(bw, bx, by, bz);
	if (bscale == 0.0) {
		return;
	}
	if (aw == 0.0 && ax == 0.0 && ay == 0.0 && az == 0.0) {
		ow = ox = oy = oz = 0.0;
		return;
	}
	const auto projected = ScaledProjection<4, REJECT>({aw, ax, ay, az}, {bw, bx, by, bz}, bscale);
	ow = projected[0];
	ox = projected[1];
	oy = projected[2];
	oz = projected[3];
}

static Vec3 Rotate(const Quat &q, const Vec3 &p) {
	Vec3 result;
	QRotate(q.w, q.x, q.y, q.z, p.x, p.y, p.z, result.x, result.y, result.z);
	return result;
}

} // namespace se3
} // namespace duckdb
