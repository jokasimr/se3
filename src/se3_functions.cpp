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
// The returned mask may share input storage: copy it before modifying its bits.
// Combine flat masks directly. Check selected components together so each row
// stops at its first NULL, without constructing a mask for every component.
static ValidityMask InputValidity(idx_t count, std::initializer_list<const Vector *> inputs) {
	ValidityMask validity(count);
	vector<VectorValidityIterator> selected;
	auto combine = [&](auto &&self, const Vector &value) -> void {
		if (value.GetVectorType() == VectorType::FLAT_VECTOR) {
			validity.Combine(FlatVector::Validity(value), count);
		} else if (value.GetVectorType() == VectorType::CONSTANT_VECTOR) {
			ValidityMask constant(count);
			constant.Combine(value, count);
			validity.Combine(constant, count);
		} else {
			auto entries = value.Validity();
			if (entries.CanHaveNull()) {
				selected.push_back(std::move(entries));
			}
		}
		if (value.GetType().InternalType() == PhysicalType::STRUCT) {
			for (const auto &child : StructVector::GetEntries(value)) {
				self(self, child);
			}
		}
	};
	for (const auto *value : inputs) {
		combine(combine, *value);
	}
	if (selected.empty() || validity.CheckAllInvalid(count)) {
		return validity;
	}

	// Flat masks may be shared with inputs; copy before marking selected rows NULL.
	ValidityMask combined(validity, count);
	// Gather each word in a register instead of updating a mask bit per NULL.
	for (idx_t base = 0; base < count; base += ValidityMask::BITS_PER_VALUE) {
		const idx_t entry = base / ValidityMask::BITS_PER_VALUE;
		const auto original = validity.GetValidityEntry(entry);
		auto word = original;
		validity_t bit = 1;
		const auto end = MinValue(count, base + ValidityMask::BITS_PER_VALUE);
		for (idx_t i = base; i < end; i++, bit <<= 1) {
			if (!(word & bit)) {
				continue;
			}
			for (const auto &entries : selected) {
				if (!entries.IsValid(i)) {
					word &= ~bit;
					break;
				}
			}
		}
		if (word != original) {
			combined.EnsureWritable();
			combined.GetValidityEntryUnsafe(entry) = word;
		}
	}
	return combined;
}

// Result buffers must be independent of the inputs before assigning validity.
static void SetResultValidity(Vector &result, const ValidityMask &validity) {
	FlatVector::SetValidity(result, validity);
	if (result.GetType().InternalType() == PhysicalType::STRUCT) {
		for (auto &child : StructVector::GetEntries(result)) {
			SetResultValidity(child, validity);
		}
	}
}

using Vec3Vector = VectorStructType<double, double, double>;
using QuatVector = VectorStructType<double, double, double, double>;
using TransformVector = VectorStructType<Vec3Vector, QuatVector>;

// Callers establish complete row validity before reading components.
static inline void ReadVec3(const VectorIterator<Vec3Vector>::ValueEntry &entry, Vec3 &value) {
	auto x = entry.GetChildValue<0>();
	auto y = entry.GetChildValue<1>();
	auto z = entry.GetChildValue<2>();
	value = {x.GetValue(), y.GetValue(), z.GetValue()};
}

static inline void ReadQuat(const VectorIterator<QuatVector>::ValueEntry &entry, Quat &value) {
	auto w = entry.GetChildValue<0>();
	auto x = entry.GetChildValue<1>();
	auto y = entry.GetChildValue<2>();
	auto z = entry.GetChildValue<3>();
	value = {w.GetValue(), x.GetValue(), y.GetValue(), z.GetValue()};
}

static inline void ReadTransform(const VectorIterator<TransformVector>::ValueEntry &entry, Vec3 &t, Quat &q) {
	ReadVec3(entry.GetChildValue<0>(), t);
	ReadQuat(entry.GetChildValue<1>(), q);
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

static bool IsFlatOrConstantAndValid(const Vector &value) {
	const auto type = value.GetVectorType();
	if (type == VectorType::CONSTANT_VECTOR) {
		if (ConstantVector::IsNull(value)) {
			return false;
		}
	} else if (type != VectorType::FLAT_VECTOR || FlatVector::Validity(value).CanHaveNull()) {
		return false;
	}
	if (value.GetType().InternalType() == PhysicalType::STRUCT) {
		for (const auto &child : StructVector::GetEntries(value)) {
			if (!IsFlatOrConstantAndValid(child)) {
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

struct SelectedQuat {
	Quat operator[](idx_t i) const {
		const auto index = selection.get_index(i);
		return {w[index], x[index], y[index], z[index]};
	}
	const double *w, *x, *y, *z;
	const SelectionVector &selection;
};

struct SelectedVec3 {
	Vec3 operator[](idx_t i) const {
		const auto index = selection.get_index(i);
		return {x[index], y[index], z[index]};
	}
	const double *x, *y, *z;
	const SelectionVector &selection;
};

static bool SharedSelection(const UnifiedVectorFormat &x, const UnifiedVectorFormat &y, const UnifiedVectorFormat &z) {
	return x.sel->data() == y.sel->data() && x.sel->data() == z.sel->data();
}

static bool SharedSelection(const UnifiedVectorFormat &w, const UnifiedVectorFormat &x, const UnifiedVectorFormat &y,
                            const UnifiedVectorFormat &z) {
	return w.sel->data() == x.sel->data() && w.sel->data() == y.sel->data() && w.sel->data() == z.sel->data();
}

// Component readers are used only after establishing complete row validity.
struct StridedDouble {
	explicit StridedDouble(const Vector &value) {
		const bool constant = value.GetVectorType() == VectorType::CONSTANT_VECTOR;
		data = constant ? ConstantVector::GetData<double>(value) : FlatVector::GetData<double>(value);
		stride = constant ? 0 : 1;
	}
	double GetValueUnsafe(idx_t i) const {
		return data[i * stride];
	}
	const double *data;
	idx_t stride;
};

template <class COMPONENT>
struct ComponentVec3 {
	explicit ComponentVec3(const Vector &value)
	    : x(StructVector::GetEntries(value)[0]), y(StructVector::GetEntries(value)[1]),
	      z(StructVector::GetEntries(value)[2]) {
	}
	Vec3 operator[](idx_t i) const {
		return {x.GetValueUnsafe(i), y.GetValueUnsafe(i), z.GetValueUnsafe(i)};
	}
	COMPONENT x, y, z;
};

template <class COMPONENT>
struct ComponentQuat {
	explicit ComponentQuat(const Vector &value)
	    : w(StructVector::GetEntries(value)[0]), x(StructVector::GetEntries(value)[1]),
	      y(StructVector::GetEntries(value)[2]), z(StructVector::GetEntries(value)[3]) {
	}
	Quat operator[](idx_t i) const {
		return {w.GetValueUnsafe(i), x.GetValueUnsafe(i), y.GetValueUnsafe(i), z.GetValueUnsafe(i)};
	}
	COMPONENT w, x, y, z;
};

using UnifiedVec3 = ComponentVec3<VectorIterator<double>>;
using UnifiedQuat = ComponentQuat<VectorIterator<double>>;
using StridedVec3 = ComponentVec3<StridedDouble>;
using StridedQuat = ComponentQuat<StridedDouble>;

struct ConstantVec3 {
	explicit ConstantVec3(const Vector &input) {
		auto values = input.Values<Vec3Vector>();
		ReadVec3(values[0], value);
	}
	Vec3 operator[](idx_t) const {
		return value;
	}
	Vec3 value;
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1], &input.data[2]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	UnifiedVectorFormat x_uf, y_uf, z_uf;
	input.data[0].ToUnifiedFormat(x_uf);
	input.data[1].ToUnifiedFormat(y_uf);
	input.data[2].ToUnifiedFormat(z_uf);

	auto *x = (const double *)x_uf.data;
	auto *y = (const double *)y_uf.data;
	auto *z = (const double *)z_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1], &input.data[2], &input.data[3]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
	const auto validity = InputValidity(count, {&a, &b});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	auto left = a.Values<Vec3Vector>();
	auto right = b.Values<Vec3Vector>();
	double *ox, *oy, *oz;
	PrepareVec3Out(result, count, ox, oy, oz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 av, bv;
		ReadVec3(left[i], av);
		ReadVec3(right[i], bv);
		ox[i] = operation(av.x, bv.x);
		oy[i] = operation(av.y, bv.y);
		oz[i] = operation(av.z, bv.z);
	}
}

template <class OP>
static void ExecuteQuatBinary(const Vector &a, const Vector &b, Vector &result, idx_t count, OP operation) {
	if (TryExecuteComponentwise(a, b, result, count, operation)) {
		return;
	}
	const auto validity = InputValidity(count, {&a, &b});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	auto left = a.Values<QuatVector>();
	auto right = b.Values<QuatVector>();
	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, count, ow, ox, oy, oz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Quat av, bv;
		ReadQuat(left[i], av);
		ReadQuat(right[i], bv);
		ow[i] = operation(av.w, bv.w);
		ox[i] = operation(av.x, bv.x);
		oy[i] = operation(av.y, bv.y);
		oz[i] = operation(av.z, bv.z);
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
	const auto validity = InputValidity(count, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	auto values = input.data[0].Values<Vec3Vector>();
	auto scales = input.data[1].Values<double>();
	double *ox, *oy, *oz;
	PrepareVec3Out(result, count, ox, oy, oz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 value;
		ReadVec3(values[i], value);
		const double scale = scales[i].GetValue();
		ox[i] = value.x * scale;
		oy[i] = value.y * scale;
		oz[i] = value.z * scale;
	}
}

void VScaleVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t count = input.size();
	if (TryExecuteComponentwise(input.data[0], input.data[1], result, count, std::multiplies<double>())) {
		return;
	}
	const auto validity = InputValidity(count, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	auto values = input.data[0].Values<QuatVector>();
	auto scales = input.data[1].Values<double>();
	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, count, ow, ox, oy, oz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Quat value;
		ReadQuat(values[i], value);
		const double scale = scales[i].GetValue();
		ow[i] = value.w * scale;
		ox[i] = value.x * scale;
		oy[i] = value.y * scale;
		oz[i] = value.z * scale;
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf, bw_uf, bx_uf, by_uf, bz_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf;
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf;
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, n);
	auto *out = FlatVector::GetDataMutable<double>(result);

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf;
	a[0].ToUnifiedFormat(ax_uf);
	a[1].ToUnifiedFormat(ay_uf);
	a[2].ToUnifiedFormat(az_uf);

	auto *ax = (const double *)ax_uf.data;
	auto *ay = (const double *)ay_uf.data;
	auto *az = (const double *)az_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &a = StructVector::GetEntries(a_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
	const auto validity = InputValidity(count, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}

	auto &a_vector = input.data[0];
	auto &b_vector = input.data[1];
	auto &a_children = StructVector::GetEntries(a_vector);
	auto &b_children = StructVector::GetEntries(b_vector);
	std::array<UnifiedVectorFormat, N> a_formats, b_formats;
	std::array<const double *, N> a_data, b_data;
	for (std::size_t component = 0; component < N; component++) {
		a_children[component].ToUnifiedFormat(a_formats[component]);
		b_children[component].ToUnifiedFormat(b_formats[component]);
		a_data[component] = UnifiedVectorFormat::GetData<double>(a_formats[component]);
		b_data[component] = UnifiedVectorFormat::GetData<double>(b_formats[component]);
	}

	result.SetVectorType(VectorType::FLAT_VECTOR);
	FlatVector::SetSize(result, count);
	auto *out = FlatVector::GetDataMutable<double>(result);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

template <bool REJECT, class LEFT_READER, class RIGHT_READER>
static void ExecuteProjection3(const LEFT_READER &a, const RIGHT_READER &b, Vector &result, idx_t count) {
	double *ox, *oy, *oz;
	PrepareVec3Out(result, count, ox, oy, oz);
	for (idx_t i = 0; i < count; i++) {
		const auto av = a[i];
		const auto bv = b[i];
		Projection3<REJECT>(av.x, av.y, av.z, bv.x, bv.y, bv.z, ox[i], oy[i], oz[i]);
	}
}

template <bool REJECT>
static bool TryExecuteDenseProjection3(const Vector &a, const Vector &b, Vector &result, idx_t count) {
	const bool a_flat = IsFlatAndValid(a);
	const bool b_flat = IsFlatAndValid(b);
	if (a_flat && b_flat) {
		ExecuteProjection3<REJECT>(FlatVec3(a), FlatVec3(b), result, count);
	} else if (!IsFlatOrConstantAndValid(a) || !IsFlatOrConstantAndValid(b)) {
		return false;
	} else if (a.GetVectorType() == VectorType::CONSTANT_VECTOR && b_flat) {
		ExecuteProjection3<REJECT>(ConstantVec3(a), FlatVec3(b), result, count);
	} else if (b.GetVectorType() == VectorType::CONSTANT_VECTOR && a_flat) {
		ExecuteProjection3<REJECT>(FlatVec3(a), ConstantVec3(b), result, count);
	} else {
		ExecuteProjection3<REJECT>(StridedVec3(a), StridedVec3(b), result, count);
	}
	return true;
}

// vproj(vec3, vec3) -> vec3
void VProjVec3Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryExecuteDenseProjection3<false>(input.data[0], input.data[1], result, n)) {
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

template <bool REJECT, class READER>
static void ExecuteProjection4(const READER &a, const READER &b, Vector &result, idx_t count) {
	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, count, ow, ox, oy, oz);
	for (idx_t i = 0; i < count; i++) {
		const auto av = a[i];
		const auto bv = b[i];
		Projection4<REJECT>(av.w, av.x, av.y, av.z, bv.w, bv.x, bv.y, bv.z, ow[i], ox[i], oy[i], oz[i]);
	}
}

// vproj(vec4, vec4) -> vec4
void VProjVec4Fn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		ExecuteProjection4<false>(FlatQuat(input.data[0]), FlatQuat(input.data[1]), result, n);
		return;
	}
	if (IsFlatOrConstantAndValid(input.data[0]) && IsFlatOrConstantAndValid(input.data[1])) {
		ExecuteProjection4<false>(StridedQuat(input.data[0]), StridedQuat(input.data[1]), result, n);
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf, bw_uf, bx_uf, by_uf, bz_uf;
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

	const bool all_valid = validity.CannotHaveNull();
	if (all_valid && SharedSelection(aw_uf, ax_uf, ay_uf, az_uf) && SharedSelection(bw_uf, bx_uf, by_uf, bz_uf)) {
		ExecuteProjection4<false>(SelectedQuat {aw, ax, ay, az, *aw_uf.sel}, SelectedQuat {bw, bx, by, bz, *bw_uf.sel},
		                          result, n);
		return;
	}

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	SetResultValidity(result, validity);
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
	if (TryExecuteDenseProjection3<true>(input.data[0], input.data[1], result, n)) {
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat ax_uf, ay_uf, az_uf, bx_uf, by_uf, bz_uf;
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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		ExecuteProjection4<true>(FlatQuat(input.data[0]), FlatQuat(input.data[1]), result, n);
		return;
	}
	if (IsFlatOrConstantAndValid(input.data[0]) && IsFlatOrConstantAndValid(input.data[1])) {
		ExecuteProjection4<true>(StridedQuat(input.data[0]), StridedQuat(input.data[1]), result, n);
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &a_v = input.data[0];
	auto &b_v = input.data[1];
	auto &a = StructVector::GetEntries(a_v);
	auto &b = StructVector::GetEntries(b_v);

	UnifiedVectorFormat aw_uf, ax_uf, ay_uf, az_uf, bw_uf, bx_uf, by_uf, bz_uf;
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

	const bool all_valid = validity.CannotHaveNull();
	if (all_valid && SharedSelection(aw_uf, ax_uf, ay_uf, az_uf) && SharedSelection(bw_uf, bx_uf, by_uf, bz_uf)) {
		ExecuteProjection4<true>(SelectedQuat {aw, ax, ay, az, *aw_uf.sel}, SelectedQuat {bw, bx, by, bz, *bw_uf.sel},
		                         result, n);
		return;
	}

	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, n, ow, ox, oy, oz);

	SetResultValidity(result, validity);
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
	const auto validity = InputValidity(count, {&axis, &angle});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	ValidityMask result_validity(validity, count);
	auto axes = axis.Values<Vec3Vector>();
	auto angles = angle.Values<double>();
	auto read_axis = [&](idx_t i, Vec3 &unit) {
		Vec3 value;
		ReadVec3(axes[i], value);
		if (value.x == 0.0 && value.y == 0.0 && value.z == 0.0) {
			return false;
		}
		Normalize3(value.x, value.y, value.z, unit.x, unit.y, unit.z);
		return true;
	};
	auto read_angle = [&](idx_t i, double &sine, double &cosine) {
		auto entry = angles[i];
		const double half = 0.5 * entry.GetValue();
		sine = std::sin(half);
		cosine = std::cos(half);
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
		read_angle(0, sine, cosine);
	}
	double *ow, *ox, *oy, *oz;
	PrepareQuatOut(result, count, ow, ox, oy, oz);
	const idx_t iterations = CONSTANT_AXIS && CONSTANT_ANGLE ? 1 : count;
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < iterations; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		if (!CONSTANT_AXIS && !read_axis(i, unit)) {
			result_validity.SetInvalid(i);
			continue;
		}
		if constexpr (!CONSTANT_ANGLE) {
			read_angle(i, sine, cosine);
		}
		ow[i] = cosine;
		ox[i] = unit.x * sine;
		oy[i] = unit.y * sine;
		oz[i] = unit.z * sine;
	}
	SetResultValidity(result, result_validity);
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

	const auto validity = InputValidity(n, {&axis, &angle});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	ValidityMask result_validity(validity, n);

	auto &axis_v = axis;
	auto &axis_ent = StructVector::GetEntries(axis_v);
	auto &th_v = angle;

	UnifiedVectorFormat ax_uf, ay_uf, az_uf, th_uf;
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

	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		const double thv = th[th_sel->get_index(i)];
		const double axv = ax[ax_sel->get_index(i)];
		const double ayv = ay[ay_sel->get_index(i)];
		const double azv = az[az_sel->get_index(i)];
		if (!AxisAngleToQuat(axv, ayv, azv, thv, ow[i], ox[i], oy[i], oz[i])) {
			result_validity.SetInvalid(i);
			continue;
		}
	}
	SetResultValidity(result, result_validity);
}

// quat_from_axis_angle(axis:vec3, th:DOUBLE) -> quat
void QuatFromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result) {
	AxisAngleQuaternion(input.data[0], input.data[1], result, input.size());
}

template <class READER>
static void MultiplyQuaternionComponents(const Vector &a, const Vector &b, Vector &result, idx_t count) {
	const READER left(a), right(b);
	double *w, *x, *y, *z;
	PrepareQuatOut(result, count, w, x, y, z);
	for (idx_t i = 0; i < count; i++) {
		const auto av = left[i];
		const auto bv = right[i];
		QMul(av.w, av.x, av.y, av.z, bv.w, bv.x, bv.y, bv.z, w[i], x[i], y[i], z[i]);
	}
}

static void MultiplyQuaternions(const Vector &a, const Vector &b, Vector &result, idx_t count) {
	if (IsFlatAndValid(a) && IsFlatAndValid(b)) {
		MultiplyQuaternionComponents<FlatQuat>(a, b, result, count);
		return;
	}
	if (IsFlatOrConstantAndValid(a) && IsFlatOrConstantAndValid(b)) {
		MultiplyQuaternionComponents<StridedQuat>(a, b, result, count);
		return;
	}
	const auto validity = InputValidity(count, {&a, &b});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	const UnifiedQuat left(a), right(b);
	double *w, *x, *y, *z;
	PrepareQuatOut(result, count, w, x, y, z);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		const auto av = left[i];
		const auto bv = right[i];
		QMul(av.w, av.x, av.y, av.z, bv.w, bv.x, bv.y, bv.z, w[i], x[i], y[i], z[i]);
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
	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &q_v = input.data[0];
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat w_uf, x_uf, y_uf, z_uf;

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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
	const auto validity = InputValidity(n, {&t_v, &q_v});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &t = StructVector::GetEntries(t_v);
	auto &q = StructVector::GetEntries(q_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf;
	UnifiedVectorFormat qw_uf, qx_uf, qy_uf, qz_uf;

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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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

	const auto validity = InputValidity(count, {&input.data[0], &input.data[1], &input.data[2]});
	if (validity.CheckAllInvalid(count)) {
		ConstantVector::SetNull(result, count_t(count));
		return;
	}
	ValidityMask result_validity(validity, count);
	// Skip invalid translations before normalizing axes or evaluating trigonometric functions.
	auto translations = input.data[0].Values<Vec3Vector>();
	auto axes = input.data[1].Values<Vec3Vector>();
	auto angles = input.data[2].Values<double>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, count, tx, ty, tz, qw, qx, qy, qz);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < count; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 translation, axis;
		auto angle = angles[i];
		ReadVec3(translations[i], translation);
		ReadVec3(axes[i], axis);
		if (!AxisAngleToQuat(axis.x, axis.y, axis.z, angle.GetValue(), qw[i], qx[i], qy[i], qz[i])) {
			result_validity.SetInvalid(i);
			continue;
		}
		tx[i] = translation.x;
		ty[i] = translation.y;
		tz[i] = translation.z;
	}
	SetResultValidity(result, result_validity);
}

static void Se3Apply(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	const auto &transform = StructVector::GetEntries(input.data[0]);
	if (validity.CannotHaveNull()) {
		std::array<UnifiedVectorFormat, 3> t, p;
		std::array<UnifiedVectorFormat, 4> q;
		const auto &tc = StructVector::GetEntries(transform[0]);
		const auto &qc = StructVector::GetEntries(transform[1]);
		const auto &pc = StructVector::GetEntries(input.data[1]);
		for (idx_t i = 0; i < 3; i++) {
			tc[i].ToUnifiedFormat(t[i]);
			pc[i].ToUnifiedFormat(p[i]);
		}
		for (idx_t i = 0; i < 4; i++) {
			qc[i].ToUnifiedFormat(q[i]);
		}
		if (SharedSelection(t[0], t[1], t[2]) && SharedSelection(q[0], q[1], q[2], q[3]) &&
		    SharedSelection(p[0], p[1], p[2])) {
			const SelectedVec3 translations {t[0].GetData<double>(), t[1].GetData<double>(), t[2].GetData<double>(),
			                                 *t[0].sel};
			const SelectedQuat rotations {q[0].GetData<double>(), q[1].GetData<double>(), q[2].GetData<double>(),
			                              q[3].GetData<double>(), *q[0].sel};
			const SelectedVec3 points {p[0].GetData<double>(), p[1].GetData<double>(), p[2].GetData<double>(),
			                           *p[0].sel};
			double *x, *y, *z;
			PrepareVec3Out(result, n, x, y, z);
			for (idx_t i = 0; i < n; i++) {
				const auto tv = translations[i];
				const auto qv = rotations[i];
				const auto pv = points[i];
				QRotate(qv.w, qv.x, qv.y, qv.z, pv.x + tv.x, pv.y + tv.y, pv.z + tv.z, x[i], y[i], z[i]);
			}
			return;
		}
	}
	const UnifiedVec3 translations(transform[0]), points(input.data[1]);
	const UnifiedQuat rotations(transform[1]);
	double *x, *y, *z;
	PrepareVec3Out(result, n, x, y, z);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		const auto t = translations[i];
		const auto p = points[i];
		const auto q = rotations[i];
		QRotate(q.w, q.x, q.y, q.z, p.x + t.x, p.y + t.y, p.z + t.z, x[i], y[i], z[i]);
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
	if (input.data[0].GetVectorType() == VectorType::CONSTANT_VECTOR && CannotHaveNullComponents(input.data[0]) &&
	    IsFlatAndValid(input.data[1])) {
		const idx_t count = input.size();
		auto transforms = input.data[0].Values<TransformVector>();
		Vec3 t;
		Quat q;
		ReadTransform(transforms[0], t, q);
		FlatVec3 points(input.data[1]);
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			auto p = points[i];
			QRotate(q.w, q.x, q.y, q.z, p.x + t.x, p.y + t.y, p.z + t.z, x[i], y[i], z[i]);
		}
		return;
	}
	if (input.data[1].GetVectorType() == VectorType::CONSTANT_VECTOR && CannotHaveNullComponents(input.data[1]) &&
	    IsFlatAndValid(input.data[0])) {
		const idx_t count = input.size();
		const auto &transform = StructVector::GetEntries(input.data[0]);
		const FlatVec3 translations(transform[0]);
		const FlatQuat rotations(transform[1]);
		const auto p = ConstantVec3(input.data[1])[0];
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			const auto t = translations[i];
			const auto q = rotations[i];
			QRotate(q.w, q.x, q.y, q.z, p.x + t.x, p.y + t.y, p.z + t.z, x[i], y[i], z[i]);
		}
		return;
	}
	if (IsFlatOrConstantAndValid(input.data[0]) && IsFlatOrConstantAndValid(input.data[1])) {
		const idx_t count = input.size();
		const auto &transform = StructVector::GetEntries(input.data[0]);
		const StridedVec3 translations(transform[0]), points(input.data[1]);
		const StridedQuat rotations(transform[1]);
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			const auto t = translations[i];
			const auto q = rotations[i];
			const auto p = points[i];
			QRotate(q.w, q.x, q.y, q.z, p.x + t.x, p.y + t.y, p.z + t.z, x[i], y[i], z[i]);
		}
		return;
	}
	Se3Apply(input, result);
}

void Se3ApplyTFn(DataChunk &input, ExpressionState &, Vector &result) {
	AddVec3(input.data[1], input.data[0], result, input.size());
}

static void Se3ApplyQ(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	auto rotations = input.data[0].Values<QuatVector>();
	auto points = input.data[1].Values<Vec3Vector>();
	double *x, *y, *z;
	PrepareVec3Out(result, n, x, y, z);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Quat q;
		Vec3 p;
		ReadQuat(rotations[i], q);
		ReadVec3(points[i], p);
		QRotate(q.w, q.x, q.y, q.z, p.x, p.y, p.z, x[i], y[i], z[i]);
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
		ReadQuat(rotations[0], q);
		FlatVec3 points(input.data[1]);
		double *x, *y, *z;
		PrepareVec3Out(result, count, x, y, z);
		for (idx_t i = 0; i < count; i++) {
			auto p = points[i];
			QRotate(q.w, q.x, q.y, q.z, p.x, p.y, p.z, x[i], y[i], z[i]);
		}
		return;
	}
	Se3ApplyQ(input, result);
}

// se3_inv(t) -> t (negation)
void Se3InvTFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (TryNegateComponents(input.data[0], result, n, 0)) {
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &t_v = input.data[0];
	auto &t = StructVector::GetEntries(t_v);

	UnifiedVectorFormat tx_uf, ty_uf, tz_uf;
	t[0].ToUnifiedFormat(tx_uf);
	t[1].ToUnifiedFormat(ty_uf);
	t[2].ToUnifiedFormat(tz_uf);

	auto *tx = (const double *)tx_uf.data;
	auto *ty = (const double *)ty_uf.data;
	auto *tz = (const double *)tz_uf.data;

	double *ox, *oy, *oz;
	PrepareVec3Out(result, n, ox, oy, oz);

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		ox[i] = -tx[tx_uf.sel->get_index(i)];
		oy[i] = -ty[ty_uf.sel->get_index(i)];
		oz[i] = -tz[tz_uf.sel->get_index(i)];
	}
}

static void Se3InvW(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	const auto validity = InputValidity(n, {&input.data[0]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	const auto &children = StructVector::GetEntries(input.data[0]);
	const UnifiedVec3 translations(children[0]);
	const UnifiedQuat rotations(children[1]);
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		const auto t = translations[i];
		const auto q = rotations[i];
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
	const bool flat = IsFlatAndValid(input.data[0]);
	if (flat || CannotHaveNullComponents(input.data[0])) {
		const auto &children = StructVector::GetEntries(input.data[0]);
		PrepareStructChildrenFlat(result, n);
		auto &output = StructVector::GetEntries(result);
		double *tx, *ty, *tz;
		PrepareVec3Out(output[0], n, tx, ty, tz);
		auto rotate_translations = [&](const auto &translations, const auto &rotations) {
			for (idx_t i = 0; i < n; i++) {
				const auto t = translations[i];
				const auto q = rotations[i];
				QRotate(q.w, q.x, q.y, q.z, t.x, t.y, t.z, tx[i], ty[i], tz[i]);
				tx[i] = -tx[i];
				ty[i] = -ty[i];
				tz[i] = -tz[i];
			}
		};
		if (flat) {
			rotate_translations(FlatVec3(children[0]), FlatQuat(children[1]));
		} else {
			rotate_translations(UnifiedVec3(children[0]), UnifiedQuat(children[1]));
		}
		// No combined NULL mask is assigned after sharing the scalar component.
		TryNegateComponents(children[1], output[1], n, 1);
		return;
	}
	Se3InvW(input, result);
}

// se3_compose overloads: result equals applying B, then A
// W = A ∘ B  with convention W(p)=R_q(p+t)

static void Se3ComposeWW(DataChunk &input, Vector &result) {
	const idx_t n = input.size();
	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	const auto &a = StructVector::GetEntries(input.data[0]);
	const auto &b = StructVector::GetEntries(input.data[1]);
	const UnifiedVec3 a_translations(a[0]), b_translations(b[0]);
	const UnifiedQuat a_rotations(a[1]), b_rotations(b[1]);
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		const auto at = a_translations[i];
		const auto bq = b_rotations[i];
		double rx, ry, rz;
		QInvRotate(bq.w, bq.x, bq.y, bq.z, at.x, at.y, at.z, rx, ry, rz);
		const auto bt = b_translations[i];
		tx[i] = bt.x + rx;
		ty[i] = bt.y + ry;
		tz[i] = bt.z + rz;
		double rw, rqx, rqy, rqz;
		const auto aq = a_rotations[i];
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

	Se3ComposeWW(input, result);
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
static void ComposeTranslationQuaternion(DataChunk &input, Vector &result) {
	const idx_t n = input.size();

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	auto translations = input.data[0].Values<Vec3Vector>();
	auto rotations = input.data[1].Values<QuatVector>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 t;
		Quat q;
		ReadVec3(translations[i], t);
		ReadQuat(rotations[i], q);
		QInvRotate(q.w, q.x, q.y, q.z, t.x, t.y, t.z, tx[i], ty[i], tz[i]);
		qw[i] = q.w;
		qx[i] = q.x;
		qy[i] = q.y;
		qz[i] = q.z;
	}
}

void Se3ComposeTQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	const bool flat = IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1]);
	if (flat || (CannotHaveNullComponents(input.data[0]) && CannotHaveNullComponents(input.data[1]))) {
		// Initialize the parent only; prepare t below and reference q.
		result.BufferMutable().SetVectorTypeOnly(VectorType::FLAT_VECTOR);
		result.BufferMutable().SetVectorSizeOnly(n);
		auto &output = StructVector::GetEntries(result);
		double *tx, *ty, *tz;
		PrepareVec3Out(output[0], n, tx, ty, tz);
		output[1].Reference(input.data[1]);
		auto rotate_translations = [&](const auto &translations, const auto &rotations) {
			for (idx_t i = 0; i < n; i++) {
				const auto t = translations[i];
				const auto q = rotations[i];
				QInvRotate(q.w, q.x, q.y, q.z, t.x, t.y, t.z, tx[i], ty[i], tz[i]);
			}
		};
		if (flat) {
			rotate_translations(FlatVec3(input.data[0]), FlatQuat(input.data[1]));
		} else {
			rotate_translations(UnifiedVec3(input.data[0]), UnifiedQuat(input.data[1]));
		}
		return;
	}
	ComposeTranslationQuaternion(input, result);
}

// se3_compose(W, q) -> W (apply q then W)
void Se3ComposeWQFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		const auto &transform = StructVector::GetEntries(input.data[0]);
		const FlatVec3 translations(transform[0]);
		const FlatQuat left_rotations(transform[1]), right_rotations(input.data[1]);
		double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
		PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
		for (idx_t i = 0; i < n; i++) {
			const auto t = translations[i];
			const auto a = left_rotations[i], b = right_rotations[i];
			QInvRotate(b.w, b.x, b.y, b.z, t.x, t.y, t.z, tx[i], ty[i], tz[i]);
			QMul(a.w, a.x, a.y, a.z, b.w, b.x, b.y, b.z, qw[i], qx[i], qy[i], qz[i]);
		}
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	auto transforms = input.data[0].Values<TransformVector>();
	auto rotations = input.data[1].Values<QuatVector>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 t;
		Quat a, b;
		ReadTransform(transforms[i], t, a);
		ReadQuat(rotations[i], b);
		QInvRotate(b.w, b.x, b.y, b.z, t.x, t.y, t.z, tx[i], ty[i], tz[i]);
		QMul(a.w, a.x, a.y, a.z, b.w, b.x, b.y, b.z, qw[i], qx[i], qy[i], qz[i]);
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	auto rotations = input.data[0].Values<QuatVector>();
	auto transforms = input.data[1].Values<TransformVector>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 t;
		Quat a, b;
		ReadQuat(rotations[i], a);
		ReadTransform(transforms[i], t, b);
		tx[i] = t.x;
		ty[i] = t.y;
		tz[i] = t.z;
		QMul(a.w, a.x, a.y, a.z, b.w, b.x, b.y, b.z, qw[i], qx[i], qy[i], qz[i]);
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

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}
	auto transforms = input.data[0].Values<TransformVector>();
	auto translations = input.data[1].Values<Vec3Vector>();
	double *tx, *ty, *tz, *qw, *qx, *qy, *qz;
	PrepareWOut(result, n, tx, ty, tz, qw, qx, qy, qz);
	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();
	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
		}
		Vec3 t, offset;
		Quat q;
		ReadTransform(transforms[i], t, q);
		ReadVec3(translations[i], offset);
		tx[i] = t.x + offset.x;
		ty[i] = t.y + offset.y;
		tz[i] = t.z + offset.z;
		qw[i] = q.w;
		qx[i] = q.x;
		qy[i] = q.y;
		qz[i] = q.z;
	}
}

// se3_compose(t, W) -> W (apply W then t)
void Se3ComposeTWFn(DataChunk &input, ExpressionState &, Vector &result) {
	const idx_t n = input.size();
	if (IsFlatAndValid(input.data[0]) && IsFlatAndValid(input.data[1])) {
		const auto &transform = StructVector::GetEntries(input.data[1]);
		const FlatVec3 translations(input.data[0]), offsets(transform[0]);
		const FlatQuat rotations(transform[1]);
		// Initialize the parent only; prepare t below and reference q.
		result.BufferMutable().SetVectorTypeOnly(VectorType::FLAT_VECTOR);
		result.BufferMutable().SetVectorSizeOnly(n);
		auto &output = StructVector::GetEntries(result);
		double *tx, *ty, *tz;
		PrepareVec3Out(output[0], n, tx, ty, tz);
		output[1].Reference(transform[1]);
		for (idx_t i = 0; i < n; i++) {
			const auto t = translations[i], offset = offsets[i];
			const auto q = rotations[i];
			double rx, ry, rz;
			QInvRotate(q.w, q.x, q.y, q.z, t.x, t.y, t.z, rx, ry, rz);
			tx[i] = offset.x + rx;
			ty[i] = offset.y + ry;
			tz[i] = offset.z + rz;
		}
		return;
	}

	const auto validity = InputValidity(n, {&input.data[0], &input.data[1]});
	if (validity.CheckAllInvalid(n)) {
		ConstantVector::SetNull(result, count_t(n));
		return;
	}

	auto &t_v = input.data[0];
	auto &w_v = input.data[1];
	auto &t2 = StructVector::GetEntries(t_v);
	auto &went = StructVector::GetEntries(w_v);
	auto &tent = StructVector::GetEntries(went[0]);
	auto &qent = StructVector::GetEntries(went[1]);

	UnifiedVectorFormat t2x_uf, t2y_uf, t2z_uf;
	UnifiedVectorFormat tx_uf, ty_uf, tz_uf, qw_uf, qx_uf, qy_uf, qz_uf;

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

	SetResultValidity(result, validity);
	const bool all_valid = validity.CannotHaveNull();

	for (idx_t i = 0; i < n; i++) {
		if (!all_valid && !validity.RowIsValid(i)) {
			continue;
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
