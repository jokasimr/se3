#pragma once

#include "duckdb.hpp"

namespace duckdb {
namespace se3 {

void Vec3CtorFn(DataChunk &input, ExpressionState &, Vector &result);
void Vec4CtorFn(DataChunk &input, ExpressionState &, Vector &result);
void VAddVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VAddVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VSubVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VSubVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VScaleVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VScaleVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VDotVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VDotVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VNorm2Vec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VNorm2Vec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VNormVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VNormVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VNormalizeVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VNormalizeVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VCrossVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VCosAngleVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VCosAngleVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VAngleVec3Fn(DataChunk &input, ExpressionState &state, Vector &result);
void VAngleVec4Fn(DataChunk &input, ExpressionState &state, Vector &result);
void VProjVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VProjVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void VRejVec3Fn(DataChunk &input, ExpressionState &, Vector &result);
void VRejVec4Fn(DataChunk &input, ExpressionState &, Vector &result);
void QuatFromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result);
void QMulFn(DataChunk &input, ExpressionState &, Vector &result);
void QConjFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3IdentityFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3MakeFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3FromAxisAngleFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ApplyFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ApplyTFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ApplyQFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3InvTFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3InvWFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeWWFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeTTFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeQTFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeTQFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeWQFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeQWFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeWTFn(DataChunk &input, ExpressionState &, Vector &result);
void Se3ComposeTWFn(DataChunk &input, ExpressionState &, Vector &result);

} // namespace se3
} // namespace duckdb
