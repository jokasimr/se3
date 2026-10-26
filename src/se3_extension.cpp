#define DUCKDB_EXTENSION_MAIN

#include "se3_extension.hpp"
#include "se3_functions.hpp"

#include "duckdb/function/scalar_function.hpp"
#include "duckdb/parser/parsed_data/create_scalar_function_info.hpp"

namespace duckdb {

using namespace se3;

// ------------------------- Types -------------------------
static LogicalType Vec3Type() {
	child_list_t<LogicalType> f;
	f.emplace_back("x", LogicalType::DOUBLE);
	f.emplace_back("y", LogicalType::DOUBLE);
	f.emplace_back("z", LogicalType::DOUBLE);
	return LogicalType::STRUCT(f);
}

static LogicalType QuatType() {
	child_list_t<LogicalType> f;
	f.emplace_back("w", LogicalType::DOUBLE);
	f.emplace_back("x", LogicalType::DOUBLE);
	f.emplace_back("y", LogicalType::DOUBLE);
	f.emplace_back("z", LogicalType::DOUBLE);
	return LogicalType::STRUCT(f);
}

static LogicalType WType() {
	child_list_t<LogicalType> f;
	f.emplace_back("t", Vec3Type());
	f.emplace_back("q", QuatType());
	return LogicalType::STRUCT(f);
}

// ------------------------- Function metadata helpers -------------------------
static FunctionDescription FunctionMeta(vector<string> parameter_names, string description, vector<string> examples,
                                        vector<string> categories, vector<LogicalType> parameter_types = {}) {
	FunctionDescription result;
	result.parameter_types = std::move(parameter_types);
	result.parameter_names = std::move(parameter_names);
	result.description = std::move(description);
	result.examples = std::move(examples);
	result.categories = std::move(categories);
	return result;
}

static void RegisterScalarFunctions(ExtensionLoader &loader, Identifier name, vector<ScalarFunction> functions,
                                    vector<FunctionDescription> descriptions) {
	ScalarFunctionSet set(std::move(name));
	if (descriptions.size() != 1 && descriptions.size() != functions.size()) {
		throw InternalException("Expected one function description or one per overload");
	}
	for (idx_t function_idx = 0; function_idx < functions.size(); function_idx++) {
		auto &function = functions[function_idx];
		auto &description = descriptions[descriptions.size() == 1 ? 0 : function_idx];
		auto &signature = function.GetSignature();
		if (signature.GetParameterCount() != description.parameter_names.size()) {
			throw InternalException("Function description has the wrong number of parameter names");
		}
		for (idx_t parameter_idx = 0; parameter_idx < signature.GetParameterCount(); parameter_idx++) {
			signature.GetParameter(parameter_idx).SetName(Identifier(description.parameter_names[parameter_idx]));
		}
		set.AddFunction(std::move(function));
	}

	CreateScalarFunctionInfo info(std::move(set));
	info.descriptions = std::move(descriptions);
	loader.RegisterFunction(std::move(info));
}

// ------------------------- Registration -------------------------
static void LoadInternal(ExtensionLoader &loader) {
	// Generic vector constructors and arithmetic
	RegisterScalarFunctions(
	    loader, "vvec",
	    {ScalarFunction("vvec", {LogicalType::DOUBLE, LogicalType::DOUBLE, LogicalType::DOUBLE}, Vec3Type(),
	                    Vec3CtorFn),
	     ScalarFunction("vvec", {LogicalType::DOUBLE, LogicalType::DOUBLE, LogicalType::DOUBLE, LogicalType::DOUBLE},
	                    QuatType(), Vec4CtorFn)},
	    {FunctionMeta({"x", "y", "z"}, "Creates a vec3 from x, y, and z components.", {"vvec(1.0, 2.0, 3.0)"},
	                  {"vector"}, {LogicalType::DOUBLE, LogicalType::DOUBLE, LogicalType::DOUBLE}),
	     FunctionMeta({"w", "x", "y", "z"}, "Creates a vec4 from w, x, y, and z components.",
	                  {"vvec(1.0, 2.0, 3.0, 4.0)"}, {"vector"},
	                  {LogicalType::DOUBLE, LogicalType::DOUBLE, LogicalType::DOUBLE, LogicalType::DOUBLE})});

	RegisterScalarFunctions(loader, "vadd",
	                        {ScalarFunction("vadd", {Vec3Type(), Vec3Type()}, Vec3Type(), VAddVec3Fn),
	                         ScalarFunction("vadd", {QuatType(), QuatType()}, QuatType(), VAddVec4Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the element-wise sum of a and b.",
	                                      {"vadd(vvec(1.0, 2.0, 3.0), vvec(4.0, 5.0, 6.0))"}, {"vector"})});

	RegisterScalarFunctions(loader, "vsub",
	                        {ScalarFunction("vsub", {Vec3Type(), Vec3Type()}, Vec3Type(), VSubVec3Fn),
	                         ScalarFunction("vsub", {QuatType(), QuatType()}, QuatType(), VSubVec4Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the element-wise difference a - b.",
	                                      {"vsub(vvec(4.0, 5.0, 6.0), vvec(1.0, 2.0, 3.0))"}, {"vector"})});

	RegisterScalarFunctions(
	    loader, "vscale",
	    {ScalarFunction("vscale", {Vec3Type(), LogicalType::DOUBLE}, Vec3Type(), VScaleVec3Fn),
	     ScalarFunction("vscale", {QuatType(), LogicalType::DOUBLE}, QuatType(), VScaleVec4Fn)},
	    {FunctionMeta({"v", "s"}, "Scales vector v by scalar s.", {"vscale(vvec(1.0, 2.0, 3.0), 2.0)"}, {"vector"})});

	RegisterScalarFunctions(loader, "vdot",
	                        {ScalarFunction("vdot", {Vec3Type(), Vec3Type()}, LogicalType::DOUBLE, VDotVec3Fn),
	                         ScalarFunction("vdot", {QuatType(), QuatType()}, LogicalType::DOUBLE, VDotVec4Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the dot product of a and b.",
	                                      {"vdot(vvec(1.0, 2.0, 3.0), vvec(4.0, 5.0, 6.0))"}, {"vector"})});

	RegisterScalarFunctions(
	    loader, "vnorm2",
	    {ScalarFunction("vnorm2", {Vec3Type()}, LogicalType::DOUBLE, VNorm2Vec3Fn),
	     ScalarFunction("vnorm2", {QuatType()}, LogicalType::DOUBLE, VNorm2Vec4Fn)},
	    {FunctionMeta({"v"}, "Returns the squared Euclidean norm of v.", {"vnorm2(vvec(1.0, 2.0, 3.0))"}, {"vector"})});

	RegisterScalarFunctions(
	    loader, "vnorm",
	    {ScalarFunction("vnorm", {Vec3Type()}, LogicalType::DOUBLE, VNormVec3Fn),
	     ScalarFunction("vnorm", {QuatType()}, LogicalType::DOUBLE, VNormVec4Fn)},
	    {FunctionMeta({"v"}, "Returns the Euclidean norm of v.", {"vnorm(vvec(1.0, 2.0, 3.0))"}, {"vector"})});

	RegisterScalarFunctions(loader, "vnormalize",
	                        {ScalarFunction("vnormalize", {Vec3Type()}, Vec3Type(), VNormalizeVec3Fn),
	                         ScalarFunction("vnormalize", {QuatType()}, QuatType(), VNormalizeVec4Fn)},
	                        {FunctionMeta({"v"}, "Returns v divided by its Euclidean norm.",
	                                      {"vnormalize(vvec(1.0, 2.0, 3.0))"}, {"vector"})});

	RegisterScalarFunctions(loader, "vcross",
	                        {ScalarFunction("vcross", {Vec3Type(), Vec3Type()}, Vec3Type(), VCrossVec3Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the 3D cross product of a and b.",
	                                      {"vcross(vvec(1.0, 0.0, 0.0), vvec(0.0, 1.0, 0.0))"}, {"vector"})});

	RegisterScalarFunctions(
	    loader, "vcos_angle",
	    {ScalarFunction("vcos_angle", {Vec3Type(), Vec3Type()}, LogicalType::DOUBLE, VCosAngleVec3Fn),
	     ScalarFunction("vcos_angle", {QuatType(), QuatType()}, LogicalType::DOUBLE, VCosAngleVec4Fn)},
	    {FunctionMeta({"a", "b"}, "Returns vdot(a, b) / (vnorm(a) * vnorm(b)) without clamping.",
	                  {"vcos_angle(vvec(1.0, 0.0, 0.0), vvec(0.0, 1.0, 0.0))"}, {"vector"})});

	RegisterScalarFunctions(loader, "vangle",
	                        {ScalarFunction("vangle", {Vec3Type(), Vec3Type()}, LogicalType::DOUBLE, VAngleVec3Fn),
	                         ScalarFunction("vangle", {QuatType(), QuatType()}, LogicalType::DOUBLE, VAngleVec4Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the angle in radians between a and b.",
	                                      {"vangle(vvec(1.0, 0.0, 0.0), vvec(0.0, 1.0, 0.0))"}, {"vector"})});

	RegisterScalarFunctions(loader, "vproj",
	                        {ScalarFunction("vproj", {Vec3Type(), Vec3Type()}, Vec3Type(), VProjVec3Fn),
	                         ScalarFunction("vproj", {QuatType(), QuatType()}, QuatType(), VProjVec4Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the projection of a onto b.",
	                                      {"vproj(vvec(2.0, 2.0, 0.0), vvec(1.0, 0.0, 0.0))"}, {"vector"})});

	RegisterScalarFunctions(loader, "vrej",
	                        {ScalarFunction("vrej", {Vec3Type(), Vec3Type()}, Vec3Type(), VRejVec3Fn),
	                         ScalarFunction("vrej", {QuatType(), QuatType()}, QuatType(), VRejVec4Fn)},
	                        {FunctionMeta({"a", "b"}, "Returns the rejection of a from b.",
	                                      {"vrej(vvec(2.0, 2.0, 0.0), vvec(1.0, 0.0, 0.0))"}, {"vector"})});

	RegisterScalarFunctions(
	    loader, "quat_from_axis_angle",
	    {ScalarFunction("quat_from_axis_angle", {Vec3Type(), LogicalType::DOUBLE}, QuatType(), QuatFromAxisAngleFn)},
	    {FunctionMeta(
	        {"axis", "angle"},
	        "Constructs a unit quaternion from axis and angle in radians. Returns NULL for a zero-length axis.",
	        {"quat_from_axis_angle(vvec(0.0, 0.0, 1.0), pi()/2.0)"}, {"quaternion"})});

	RegisterScalarFunctions(loader, "qmul", {ScalarFunction("qmul", {QuatType(), QuatType()}, QuatType(), QMulFn)},
	                        {FunctionMeta({"qA", "qB"}, "Multiplies quaternions qA and qB.",
	                                      {"qmul(quat_from_axis_angle(vvec(0.0, 0.0, 1.0), pi()/2.0), "
	                                       "quat_from_axis_angle(vvec(0.0, 1.0, 0.0), pi()/2.0))"},
	                                      {"quaternion"})});

	RegisterScalarFunctions(loader, "qconj", {ScalarFunction("qconj", {QuatType()}, QuatType(), QConjFn)},
	                        {FunctionMeta({"q"}, "Returns the quaternion conjugate of q.",
	                                      {"qconj(vvec(1.0, 2.0, 3.0, 4.0))"}, {"quaternion"})});

	RegisterScalarFunctions(
	    loader, "qnorm2", {ScalarFunction("qnorm2", {QuatType()}, LogicalType::DOUBLE, VNorm2Vec4Fn)},
	    {FunctionMeta({"q"}, "Returns the squared norm of quaternion q.",
	                  {"qnorm2(quat_from_axis_angle(vvec(0.0, 0.0, 1.0), pi()/2.0))"}, {"quaternion"})});

	RegisterScalarFunctions(loader, "se3_identity", {ScalarFunction("se3_identity", {}, WType(), Se3IdentityFn)},
	                        {FunctionMeta({}, "Returns the identity transform.", {"se3_identity()"}, {"se3"})});

	RegisterScalarFunctions(
	    loader, "se3_make", {ScalarFunction("se3_make", {Vec3Type(), QuatType()}, WType(), Se3MakeFn)},
	    {FunctionMeta({"t", "q"}, "Constructs a transform from translation t and quaternion q.",
	                  {"se3_make(vvec(1.0, 2.0, 3.0), quat_from_axis_angle(vvec(0.0, 0.0, 1.0), pi()/2.0))"},
	                  {"se3"})});

	RegisterScalarFunctions(
	    loader, "se3_from_axis_angle",
	    {ScalarFunction("se3_from_axis_angle", {Vec3Type(), Vec3Type(), LogicalType::DOUBLE}, WType(),
	                    Se3FromAxisAngleFn)},
	    {FunctionMeta({"t", "axis", "angle"},
	                  "Constructs a transform from translation t and an axis-angle rotation. Returns NULL for a "
	                  "zero-length axis.",
	                  {"se3_from_axis_angle(vvec(1.0, 0.0, 0.0), vvec(0.0, 0.0, 1.0), pi()/2.0)"}, {"se3"})});

	RegisterScalarFunctions(
	    loader, "se3_apply",
	    {ScalarFunction("se3_apply", {WType(), Vec3Type()}, Vec3Type(), Se3ApplyFn),
	     ScalarFunction("se3_apply", {Vec3Type(), Vec3Type()}, Vec3Type(), Se3ApplyTFn),
	     ScalarFunction("se3_apply", {QuatType(), Vec3Type()}, Vec3Type(), Se3ApplyQFn)},
	    {FunctionMeta({"W", "p"},
	                  "Applies transformation W to point p. W can be a full transformation struct, a vec3 representing "
	                  "only the translation part, or a quat representing only the rotation part.",
	                  {"se3_apply(se3_identity(), vvec(1.0, 2.0, 3.0))"}, {"se3"})});

	RegisterScalarFunctions(
	    loader, "se3_inv",
	    {ScalarFunction("se3_inv", {Vec3Type()}, Vec3Type(), Se3InvTFn),
	     ScalarFunction("se3_inv", {QuatType()}, QuatType(), QConjFn),
	     ScalarFunction("se3_inv", {WType()}, WType(), Se3InvWFn)},
	    {FunctionMeta({"W"},
	                  "Returns the inverse transformation of W. W can be a full transformation struct, a vec3 "
	                  "representing only the translation part, or a quat representing only the rotation part.",
	                  {"se3_inv(se3_identity())"}, {"se3"})});

	RegisterScalarFunctions(
	    loader, "se3_compose",
	    {ScalarFunction("se3_compose", {WType(), WType()}, WType(), Se3ComposeWWFn),
	     ScalarFunction("se3_compose", {Vec3Type(), Vec3Type()}, Vec3Type(), Se3ComposeTTFn),
	     ScalarFunction("se3_compose", {QuatType(), QuatType()}, QuatType(), QMulFn),
	     ScalarFunction("se3_compose", {QuatType(), Vec3Type()}, WType(), Se3ComposeQTFn),
	     ScalarFunction("se3_compose", {Vec3Type(), QuatType()}, WType(), Se3ComposeTQFn),
	     ScalarFunction("se3_compose", {WType(), QuatType()}, WType(), Se3ComposeWQFn),
	     ScalarFunction("se3_compose", {QuatType(), WType()}, WType(), Se3ComposeQWFn),
	     ScalarFunction("se3_compose", {WType(), Vec3Type()}, WType(), Se3ComposeWTFn),
	     ScalarFunction("se3_compose", {Vec3Type(), WType()}, WType(), Se3ComposeTWFn)},
	    {FunctionMeta({"W2", "W1"},
	                  "Composes two transformations, applying W1 first and then W2. W1 and W2 can each be a full "
	                  "transformation struct, a vec3 representing only the translation part, or a quat representing "
	                  "only the rotation part.",
	                  {"se3_compose(se3_identity(), se3_identity())"}, {"se3"})});
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
