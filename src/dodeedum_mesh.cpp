#include "../include/dodeedum_mesh.h"
#include "glm/gtc/type_precision.hpp"
#include <stdexcept>


uint32_t DoDeeDum::MeshIndices::get_triangle_count() const
{
    switch (geometry_type) {
        case GeomType::TRIANGLES:
            return (uint32_t)(no_indices / 3);
            
        case GeomType::TRIANGLE_STRIP:
            return (no_indices >= 3) ? (uint32_t)(no_indices - 2) : 0;
            
        case GeomType::TRIANGLE_FAN:
            return (no_indices >= 3) ? (uint32_t)(no_indices - 2) : 0;
            
        default:
            return 0;
    }
}

uint32_t DoDeeDum::MeshIndices::read_index(uint32_t index) const
{
    switch (index_type) {
        case AttribType::UNSIGNED_BYTE: {
            const uint8_t* buf = (const uint8_t*)index_array_buffer;
            return buf[index];
        }
        case AttribType::UNSIGNED_SHORT: {
            const uint16_t* buf = (const uint16_t*)index_array_buffer;
            return buf[index];
        }
        case AttribType::UNSIGNED_INT: {
            const uint32_t* buf = (const uint32_t*)index_array_buffer;
            return buf[index];
        }
        default:
            return 0;
    }
}

glm::uvec3 DoDeeDum::MeshIndices::get_tri(uint32_t tri_id) const
{
	glm::uvec3 vertex_indices{0};
	
	switch (geometry_type) {
        case GeomType::TRIANGLES: {
            // Simple case: each triangle uses 3 consecutive indices
           uint64_t base_index = tri_id * 3;
            if (base_index + 2 >= no_indices) {
                throw std::out_of_range("index");
            }
            
            if (index_array_buffer) {
                // Indexed geometry
                vertex_indices[0] = read_index(base_index);
                vertex_indices[1] = read_index(base_index + 1);
                vertex_indices[2] = read_index(base_index + 2);
            } else {
                // Non-indexed geometry
                vertex_indices[0] = base_index;
                vertex_indices[1] = base_index + 1;
                vertex_indices[2] = base_index + 2;
            }
            break;
        }
        
        case GeomType::TRIANGLE_STRIP: {
            // Triangle strip: triangles share vertices
            // Triangle 0: vertices 0,1,2
            // Triangle 1: vertices 1,2,3 (or 1,3,2 for correct winding)
            // Triangle 2: vertices 2,3,4
            // etc.
           uint64_t base_index = tri_id;
            if (base_index + 2 >= no_indices) {
                throw std::out_of_range("index");
            }
            
            if (index_array_buffer) {
                vertex_indices[0] = read_index(base_index);
                vertex_indices[1] = read_index(base_index + 1);
                vertex_indices[2] = read_index(base_index + 2);
            } else {
                vertex_indices[0] = base_index;
                vertex_indices[1] = base_index + 1;
                vertex_indices[2] = base_index + 2;
            }
            
            // Swap vertices 1 and 2 for odd triangles to maintain consistent winding
            if (tri_id & 1) {
                uint64_t temp = vertex_indices[1];
                vertex_indices[1] = vertex_indices[2];
                vertex_indices[2] = temp;
            }
            break;
        }
        
        case GeomType::TRIANGLE_FAN: {
            // Triangle fan: all triangles share the first vertex
            // Triangle 0: vertices 0,1,2
            // Triangle 1: vertices 0,2,3
            // Triangle 2: vertices 0,3,4
            // etc.
            if (tri_id + 2 >= no_indices) {
                throw std::out_of_range("index");
            }
            
            if (index_array_buffer) {
                vertex_indices[0] = read_index(0);
                vertex_indices[1] = read_index(tri_id + 1);
                vertex_indices[2] = read_index(tri_id + 2);
            } else {
                vertex_indices[0] = 0;
                vertex_indices[1] = tri_id + 1;
                vertex_indices[2] = tri_id + 2;
            }
            break;
        }
        
        default:
            throw std::bad_cast();
    }
    
    // Validate vertex indices
    for (int i = 0; i < 3; i++) {
        if (vertex_indices[i] >= no_verts) {
            throw std::out_of_range("vertex");
        }
    }
    
    return vertex_indices;
}

size_t		 DoDeeDum::MeshAttrib::get_type_size() const
{
    switch (type) {
        case AttribType::BYTE:
        case AttribType::UNSIGNED_BYTE:
            return 1;
        case AttribType::SHORT:
        case AttribType::UNSIGNED_SHORT:
        case AttribType::HALF_FLOAT:
            return 2;
        case AttribType::INT:
        case AttribType::UNSIGNED_INT:
        case AttribType::FLOAT:
        case AttribType::FIXED:
        case AttribType::INT_2_10_10_10_REV:
        case AttribType::UNSIGNED_INT_2_10_10_10_REV:
        case AttribType::UNSIGNED_INT_10F_11F_11F_REV:
            return 4;
        case AttribType::DOUBLE:
            return 8;
        default:
            return 0;
    }
}

// Helper to convert half float to float
static float half_to_float(uint16_t h) {
    uint32_t sign = (h & 0x8000) << 16;
    uint32_t exponent = (h & 0x7C00) >> 10;
    uint32_t mantissa = h & 0x03FF;
    
    if (exponent == 0) {
        if (mantissa == 0) return *(float*)&sign; // ±0
        // Subnormal
        exponent = 1;
        while (!(mantissa & 0x0400)) {
            mantissa <<= 1;
            exponent--;
        }
        mantissa &= 0x03FF;
    } else if (exponent == 31) {
        // Inf or NaN
        exponent = 255;
    }
    
    exponent = exponent + 112;
    mantissa <<= 13;
    
    uint32_t result = sign | (exponent << 23) | mantissa;
    return *(float*)&result;
}

glm::dvec4   DoDeeDum::MeshAttrib::readf(uint32_t index) const
{	
    if (!src) {
		throw std::invalid_argument("src cannot be null");
    }
    
    if (size < 1 || size > 4) {
        throw std::invalid_argument("component size");
    }
        
    size_t type_size = get_type_size();
    if (type_size == 0) {
        throw std::bad_cast();
    }
    
    size_t stride = this->stride;
    if (stride == 0) {
        stride = type_size * size;
    }
    
    size_t offset = this->offset + (index * stride);
    size_t required_size = offset + (type_size * size);
    
    if (required_size > byteLength) {
        throw std::out_of_range("index");
    }
    
    uint8_t* base = (uint8_t*)src + offset;
    glm::dvec4 dst{0};
    
	switch (type) {
        default:
            throw std::bad_cast();
		case AttribType::FLOAT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((float*)(base))[i];
            }
        } break;
		case AttribType::DOUBLE: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((double*)(base))[i];
            }
        } break;
		case AttribType::HALF_FLOAT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = half_to_float(((uint16_t*)(base))[i]);
            }
        } break;
		case AttribType::BYTE: 
		{
			for (int i = 0; i < size; i++) {
				int64_t eax = ((int8_t*)(base))[i];
                dst[i] = normalized? ((double)eax / 127.0) : (double)eax;
            }
        } break;
		case AttribType::SHORT: 
		{
			for (int i = 0; i < size; i++) {
				int64_t eax = ((int16_t*)(base))[i];
                dst[i] = normalized? ((double)eax / 32767.0) : (double)eax;
            }
        } break;
		case AttribType::INT: 
		{
			for (int i = 0; i < size; i++) {
				int64_t eax = ((int32_t*)(base))[i];
                dst[i] = normalized? ((double)eax / 2147483647.0) : (double)eax;
            }
        } break;
        case AttribType::UNSIGNED_BYTE: 
		{
			for (int i = 0; i < size; i++) {
				int64_t eax = ((uint8_t*)(base))[i];
                dst[i] = normalized? ((double)eax / 255.0) : (double)eax;
            }
        } break;
		case AttribType::UNSIGNED_SHORT: 
		{
			for (int i = 0; i < size; i++) {
				int64_t eax = ((uint16_t*)(base))[i];
                dst[i] = normalized? ((double)eax / 65535.0) : (double)eax;
            }
        } break;
		case AttribType::UNSIGNED_INT: 
		{
			for (int i = 0; i < size; i++) {
				int64_t eax = ((uint32_t*)(base))[i];
                dst[i] = normalized? ((double)eax / 4294967295.0) : (double)eax;
            }
        } break;
	 // Special packed formats (only when size matches expected)
		case AttribType::INT_2_10_10_10_REV:
		case AttribType::UNSIGNED_INT_2_10_10_10_REV: {
			float tmp[4];			
			
			uint32_t packed = *(uint32_t*)base;
			if (type == AttribType::INT_2_10_10_10_REV) {
				// Signed
				int32_t x = (int32_t)(packed << 22) >> 22;  // Sign extend
				int32_t y = (int32_t)(packed << 12) >> 22;
				int32_t z = (int32_t)(packed << 2) >> 22;
				int32_t w = (int32_t)(packed >> 30) & 0x3;
				
				tmp[0] = (float)(normalized ? (x / 511.0) : x);
				tmp[1] = (float)(normalized ? (y / 511.0) : y);
				tmp[2] = (float)(normalized ? (z / 511.0) : z);
				tmp[3] = (float)(normalized ? (w / 1.0) : w);
			} else {
				// Unsigned
				uint32_t x = packed & 0x3FF;
				uint32_t y = (packed >> 10) & 0x3FF;
				uint32_t z = (packed >> 20) & 0x3FF;
				uint32_t w = (packed >> 30) & 0x3;
				
				tmp[0] = (float)(normalized ? (x / 1023.0) : x);
				tmp[1] = (float)(normalized ? (y / 1023.0) : y);
				tmp[2] = (float)(normalized ? (z / 1023.0) : z);
				tmp[3] = (float)(normalized ? (w / 3.0) : w);
			}
				
			for (int i = 0; i < size; i++) {
                dst[i] = tmp[i];
            }
		} break;
    }
    
    return dst;
}

glm::i64vec4 DoDeeDum::MeshAttrib::readi(uint32_t index) const
{
    if (!src) {
		throw std::invalid_argument("src cannot be null");
    }
    
    if (size < 1 || size > 4) {
        throw std::invalid_argument("component size");
    }
        
    size_t type_size = get_type_size();
    if (type_size == 0) {
        throw std::bad_cast();
    }
    
    size_t stride = this->stride;
    if (stride == 0) {
        stride = type_size * size;
    }
    
    size_t offset = this->offset + (index * stride);
    size_t required_size = offset + (type_size * size);
    
    if (required_size > byteLength) {
        throw std::out_of_range("index");
    }
    
    uint8_t* base = (uint8_t*)src + offset;
    
    glm::i64vec4 dst{0};
    
	switch (type) {
        default:
            throw std::bad_cast();
		case AttribType::FLOAT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = (int32_t)((float*)(base))[i];
            }
        } break;
		case AttribType::DOUBLE: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = (int32_t)((double*)(base))[i];
            }
        } break;
		case AttribType::HALF_FLOAT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = (int32_t)half_to_float(((uint16_t*)(base))[i]);
            }
        } break;
		case AttribType::BYTE: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((int8_t*)(base))[i];
            }
        } break;
		case AttribType::SHORT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((int16_t*)(base))[i];
            }
        } break;
		case AttribType::INT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((int32_t*)(base))[i];
            }
        } break;
        case AttribType::UNSIGNED_BYTE: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((uint8_t*)(base))[i];
            }
        } break;
		case AttribType::UNSIGNED_SHORT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = ((uint16_t*)(base))[i];
            }
        } break;
		case AttribType::UNSIGNED_INT: 
		{
			for (int i = 0; i < size; i++) {
                dst[i] = (int32_t)((uint32_t*)(base))[i];
            }
        } break;
	 // Special packed formats (only when size matches expected)
		case AttribType::INT_2_10_10_10_REV:
		case AttribType::UNSIGNED_INT_2_10_10_10_REV: {
			int32_t tmp[4];			
			
			uint32_t packed = *(uint32_t*)base;
			if (type == AttribType::INT_2_10_10_10_REV) {
				// Signed
				tmp[0] = (int32_t)(packed << 22) >> 22;  // Sign extend
				tmp[1] = (int32_t)(packed << 12) >> 22;
				tmp[2] = (int32_t)(packed << 2) >> 22;
				tmp[3] = (int32_t)(packed >> 30) & 0x3;
			} else {
				// Unsigned
				tmp[0] = (int32_t)(packed & 0x3FF);
				tmp[1] = (int32_t)((packed >> 10) & 0x3FF);
				tmp[2] = (int32_t)((packed >> 20) & 0x3FF);
				tmp[3] = (int32_t)((packed >> 30) & 0x3);
			}
				
			for (int i = 0; i < size; i++) {
                dst[i] = tmp[i];
            }
		} break;
    }
    
    
    return dst;
}
