#ifndef DoDeeDum_SKINNING_H
#define DoDeeDum_SKINNING_H
#include "dodeedum_mesh.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <span>
#include <functional>

namespace DoDeeDum
{

// Dual quaternion representation: [real, dual]
// real quaternion handles rotation
// dual quaternion handles translation
struct DualQuat
{
	glm::quat real;  // rotation
	glm::quat dual;  // translation (encoded)
	
	DualQuat() : real(1.0, 0.0, 0.0, 0.0), dual(0.0, 0.0, 0.0, 0.0) {}
	DualQuat(glm::quat const& r, glm::quat const& d) : real(r), dual(d) {}
	
	// Construct from rotation quaternion and translation vector
	static DualQuat from_rotation_translation(glm::quat const& rotation, glm::vec3 const& translation);
	
	// Construct from a 4x4 matrix (extracts rotation and translation only)
	explicit DualQuat(glm::mat4 const& mat);
	
	// Convert back to a 4x4 matrix
	explicit operator glm::mat4() const;
	
	// Normalize the dual quaternion
	DualQuat normalized() const;
	
	// Blend two dual quaternions with weight t ∈ [0,1]
	// Handles antipodality (shortest path)
	static DualQuat blend(DualQuat const& a, DualQuat const& b, float t);
};

// Linear blend skinning using matrices
// This is the classic approach - simple but can have candy-wrapper artifacts
struct MatrixSkinning
{
	std::span<const glm::mat4> bone_transforms;
	
	// Get the skinned transform for a vertex
	glm::mat4 operator()(Primitive const& primitive, uint32_t vertex) const;
	
	// Helper: blend matrices linearly based on skinning weights
	static glm::mat4 blend_matrices(
			std::span<const glm::mat4> transforms,
			std::span<const Vert::Skinning> skinning
			);
};

// Dual quaternion skinning with separate scale
// Smoother blending, no candy-wrapper artifacts, supports non-uniform scale
struct DualQuatScaleSkinning
{
	std::span<const DualQuat> bone_dualquats;
	std::span<const glm::vec3> bone_scales;
	
	// Get the skinned transform for a vertex
	glm::mat4 operator()(Primitive const& primitive, uint32_t vertex) const;
	
	// Helper: blend dual quaternions based on skinning weights
	static DualQuat blend_dualquats(
		std::span<const DualQuat> dualquats,
		std::span<const Vert::Skinning> skinning
	);
	
	// Helper: blend scales linearly
	static glm::vec3 blend_scales(
			std::span<const glm::vec3> scales,
			std::span<const Vert::Skinning> skinning
			);
};

// Utility functions for building bone transforms from skeleton hierarchies

// Compute final bone transforms: current_pose * inverse_bind_pose
// Used for matrix skinning
std::vector<glm::mat4> compute_bone_matrices(
	std::span<const glm::mat4> current_pose,
	std::span<const glm::mat4> inverse_bind_pose
);

// Decompose matrices into dual quaternions and scales
// Used for dual quaternion + scale skinning
void decompose_bone_matrices(
	std::span<const glm::mat4> bone_matrices,
	std::vector<DualQuat>& out_dualquats,
	std::vector<glm::vec3>& out_scales
);

// Extract scale from a matrix (using polar decomposition approximation)
glm::vec3 extract_scale(glm::mat4 const& mat);

// Remove scale from a matrix, leaving only rotation and translation
glm::mat4 remove_scale(glm::mat4 const& mat);

}

#endif // DoDeeDum_SKINNING_H
