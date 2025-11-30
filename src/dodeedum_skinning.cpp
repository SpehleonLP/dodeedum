#include "dodeedum_skinning.h"
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <stdexcept>

// ============================================================================
// DualQuat Implementation
// ============================================================================

DoDeeDum::DualQuat DoDeeDum::DualQuat::from_rotation_translation(glm::quat const& rotation, glm::vec3 const& translation)
{
	DualQuat result;
	result.real = rotation;
	
	// dual = 0.5 * translation_quat * real
	// where translation_quat = (0, translation.x, translation.y, translation.z)
	glm::quat trans_quat(0.0, translation.x, translation.y, translation.z);
	result.dual = 0.5f * trans_quat * rotation;
	
	return result;
}

DoDeeDum::DualQuat::DualQuat(glm::mat4 const& mat)
{
	glm::vec3 scale;
	glm::quat rotation;
	glm::vec3 translation;
	glm::vec3 skew;
	glm::vec4 perspective;
	if(glm::decompose(mat, scale, rotation, translation, skew, perspective) == 0)
		throw std::invalid_argument("Inverse bind pose invalid (cannot be decomposed).");

	*this = from_rotation_translation(rotation, translation);
}

DoDeeDum::DualQuat::operator glm::mat4() const
{
	// Convert rotation quaternion to matrix
	glm::mat4 result = glm::mat4_cast(real);
	
	// Extract translation from dual quaternion
	// translation = 2 * dual * conjugate(real)
	glm::quat real_conj = glm::conjugate(real);
	glm::quat trans_quat = 2.0f * dual * real_conj;
	
	result[3][0] = trans_quat.x;
	result[3][1] = trans_quat.y;
	result[3][2] = trans_quat.z;
	result[3][3] = 1.0;
	
	return result;
}

DoDeeDum::DualQuat DoDeeDum::DualQuat::normalized() const
{
	float len = glm::length(real);
	if (len < 1e-10)
		return DualQuat();
	
	DualQuat result;
	result.real = real / len;
	result.dual = dual / len;
	return result;
}

DoDeeDum::DualQuat DoDeeDum::DualQuat::blend(DualQuat const& a, DualQuat const& b, float t)
{
	DualQuat result;
	
	// Handle antipodality: ensure we take the shortest path
	// Check if quaternions are on opposite hemispheres
	float dot = glm::dot(a.real, b.real);
	
	DualQuat b_corrected = b;
	if (dot < 0.0)
	{
		// Flip b to take shorter path
		b_corrected.real = -b.real;
		b_corrected.dual = -b.dual;
	}
	
	// Linear interpolation
	result.real = (1.f - t) * a.real + t * b_corrected.real;
	result.dual = (1.f - t) * a.dual + t * b_corrected.dual;
	
	// Normalize to maintain unit dual quaternion property
	return result.normalized();
}

// ============================================================================
// MatrixSkinning Implementation
// ============================================================================

glm::mat4 DoDeeDum::MatrixSkinning::operator()(Primitive const& primitive, uint32_t vertex) const
{
	if (bone_transforms.empty())
		return glm::mat4(1.0);
	
	// Get skinning data for this vertex
	std::vector<Vert::Skinning> skinning;
	skinning.reserve(16);
	for (auto const& sk : primitive.skin)
	{
		auto joints = sk.joints(vertex);
		auto weights = sk.weights(vertex);
		
		for (int i = 0; i < 4; ++i)
		{
			if (weights[i] > 0.0)
			{
				skinning.push_back({.joint = joints[i], .weight = weights[i]});
			}
		}
	}
	
	if (skinning.empty())
		return glm::mat4(1.0);
	
	return blend_matrices(bone_transforms, skinning);
}

glm::mat4 DoDeeDum::MatrixSkinning::blend_matrices(
	std::span<const glm::mat4>  transforms,
	std::span<const Vert::Skinning> skinning)
{
	glm::mat4 result(0.0);
	
	for (auto const& skin : skinning)
	{
		if (skin.joint >= 0 && skin.joint < static_cast<int64_t>(transforms.size()))
		{
			result += float(skin.weight) * transforms[skin.joint];
		}
	}
	
	return result;
}

// ============================================================================
// DualQuatScaleSkinning Implementation
// ============================================================================

glm::mat4 DoDeeDum::DualQuatScaleSkinning::operator()(Primitive const& primitive, uint32_t vertex) const
{
	if (bone_dualquats.empty())
		return glm::mat4(1.0);
	
	// Get skinning data for this vertex
	std::vector<Vert::Skinning> skinning;
	for (auto const& sk : primitive.skin)
	{
		auto joints = sk.joints(vertex);
		auto weights = sk.weights(vertex);
		
		for (int i = 0; i < 4; ++i)
		{
			if (weights[i] > 0.0)
			{
				skinning.push_back({.joint = joints[i], .weight = weights[i]});
			}
		}
	}
	
	if (skinning.empty())
		return glm::mat4(1.0);
	
	// Blend dual quaternions
	DualQuat blended_dq = blend_dualquats(bone_dualquats, skinning);
	
	// Blend scales
	glm::vec3 blended_scale = blend_scales(bone_scales, skinning);
	
	// Convert to matrix
	glm::mat4 result = glm::mat4(blended_dq);
	
	// Apply scale
	result[0] *= blended_scale.x;
	result[1] *= blended_scale.y;
	result[2] *= blended_scale.z;
	
	return result;
}

DoDeeDum::DualQuat DoDeeDum::DualQuatScaleSkinning::blend_dualquats(
	std::vector<DualQuat> const& dualquats,
	std::vector<Vert::Skinning> const& skinning)
{
	if (skinning.empty())
		return DualQuat();
	
	// Start with the first weighted dual quaternion
	int64_t first_joint = skinning[0].joint;
	if (first_joint < 0 || first_joint >= static_cast<int64_t>(dualquats.size()))
		return DualQuat();
	
	DualQuat result = dualquats[first_joint];
	double total_weight = skinning[0].weight;
	
	// Blend with remaining dual quaternions
	for (size_t i = 1; i < skinning.size(); ++i)
	{
		int64_t joint = skinning[i].joint;
		if (joint < 0 || joint >= static_cast<int64_t>(dualquats.size()))
			continue;
		
		double weight = skinning[i].weight;
		double t = weight / (total_weight + weight);
		
		result = DualQuat::blend(result, dualquats[joint], t);
		total_weight += weight;
	}
	
	return result.normalized();
}

glm::vec3 DoDeeDum::DualQuatScaleSkinning::blend_scales(
	std::span<const glm::vec3> scales,
	std::span<const Vert::Skinning> skinning)
{
	glm::vec3 result(0.0);
	
	for (auto const& skin : skinning)
	{
		if (skin.joint >= 0 && skin.joint < static_cast<int64_t>(scales.size()))
		{
			result += float(skin.weight) * scales[skin.joint];
		}
	}
	
	return result;
}

// ============================================================================
// Utility Functions
// ============================================================================

std::vector<glm::mat4> DoDeeDum::compute_bone_matrices(
	std::span<const glm::mat4> current_pose,
	std::span<const glm::mat4> inverse_bind_pose)
{
	size_t count = std::min(current_pose.size(), inverse_bind_pose.size());
	std::vector<glm::mat4> result(count);
	
	for (size_t i = 0; i < count; ++i)
	{
		result[i] = current_pose[i] * inverse_bind_pose[i];
	}
	
	return result;
}

glm::vec3 DoDeeDum::extract_scale(glm::mat4 const& mat)
{
	// Extract scale as length of basis vectors
	glm::vec3 scale;
	scale.x = glm::length(glm::vec3(mat[0]));
	scale.y = glm::length(glm::vec3(mat[1]));
	scale.z = glm::length(glm::vec3(mat[2]));
	
	// Handle reflection (negative determinant)
	glm::vec3 cross_product = glm::cross(glm::vec3(mat[0]), glm::vec3(mat[1]));
	if (glm::dot(cross_product, glm::vec3(mat[2])) < 0.0)
	{
		scale.x = -scale.x;
	}
	
	return scale;
}

glm::mat4 DoDeeDum::remove_scale(glm::mat4 const& mat)
{
	glm::vec3 scale = extract_scale(mat);
	
	glm::mat4 result = mat;
	
	// Remove scale from basis vectors
	if (std::abs(scale.x) > 1e-10)
		result[0] /= scale.x;
	if (std::abs(scale.y) > 1e-10)
		result[1] /= scale.y;
	if (std::abs(scale.z) > 1e-10)
		result[2] /= scale.z;
	
	return result;
}

void DoDeeDum::decompose_bone_matrices(
	std::span<const glm::mat4> bone_matrices,
	std::vector<DualQuat>& out_dualquats,
	std::vector<glm::vec3>& out_scales)
{
	out_dualquats.resize(bone_matrices.size());
	out_scales.resize(bone_matrices.size());
	
	for (size_t i = 0; i < bone_matrices.size(); ++i)
	{
		// Extract scale
		out_scales[i] = extract_scale(bone_matrices[i]);
		
		// Remove scale to get rotation + translation only
		glm::mat4 rot_trans = remove_scale(bone_matrices[i]);
		
		// Convert to dual quaternion
		out_dualquats[i] = DualQuat(rot_trans);
	}
}


