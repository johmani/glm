#pragma once

#include <glm/glm.hpp>

namespace glm {

    template <typename Vec>
	struct box
	{
		using T = typename Vec::value_type;
		static constexpr int n = Vec().length(); // Dimension (e.g., 2 or 3)

		Vec min, max;

		box() = default;

		box(const Vec& min, const Vec& max) : min(min), max(max) {}

		box(int numPoints, const Vec* points)
		{
			if (numPoints == 0) {
				min = Vec(std::numeric_limits<T>::max());
				max = Vec(std::numeric_limits<T>::lowest());
				return;
			}
			min = points[0];
			max = points[0];
			for (int i = 1; i < numPoints; ++i) 
			{
				min = glm::min(min, points[i]);
				max = glm::max(max, points[i]);
			}
		}

		template<typename U>
		box(const box<glm::vec<n, U, glm::defaultp>>& b)
			: min(b.min), max(b.max) {
		}

		static box empty() { return box(Vec(std::numeric_limits<T>::max()), Vec(std::numeric_limits<T>::lowest())); }
		bool isEmpty() const { return glm::any(glm::greaterThan(min, max)); }
		bool contains(const Vec& point) const { return glm::all(glm::lessThanEqual(min, point)) && glm::all(glm::lessThanEqual(point, max)); }
		// By convention, an empty box is contained in every box.
		bool contains(const box& other) const { return other.isEmpty() || (glm::all(glm::lessThanEqual(min, other.min)) && glm::all(glm::lessThanEqual(other.max, max))); }
		bool intersects(const box& other) const { return glm::all(glm::lessThanEqual(other.min, max)) && glm::all(glm::lessThanEqual(min, other.max)); }

		// Returns true if all components of the box are finite.
		bool isFinite() const { return glm::all(glm::isfinite(min)) && glm::all(glm::isfinite(max)); }

		// Clamps a point to the bounds of the box.
		Vec clamp(const Vec& point) const { return glm::clamp(point, min, max); }

		// Returns the center of the box.
		Vec center() const { return min + (max - min) / T(2); }

		// Returns the diagonal vector (max - min).
		Vec diagonal() const { return max - min; }

		// Returns one of the 2^n corners of the box.
		// The index iCorner should be in the range [0, 2^n).
		Vec getCorner(int iCorner) const {
			Vec corner(0);
			for (int j = 0; j < n; ++j) {
				// If bit j of iCorner is set, choose max[j], otherwise choose min[j].
				corner[j] = (iCorner & (1 << j)) ? max[j] : min[j];
			}
			return corner;
		}

		// Fills an array (of length 2^n) with all the box corners.
		void getCorners(Vec* cornersOut) const {
			int numCorners = 1 << n;
			for (int i = 0; i < numCorners; ++i)
				cornersOut[i] = getCorner(i);
		}

		// Computes the extents (min and max dot products) of the box along a given axis.
		void getExtentsAlongAxis(const Vec& axis, T& outMin, T& outMax) const {
			T dotCenter = glm::dot(center(), axis);
			// For the diagonal, we use the absolute value of the axis components.
			T dotDiagonal = glm::dot(diagonal(), glm::abs(axis));
			outMin = dotCenter - dotDiagonal;
			outMax = dotCenter + dotDiagonal;
		}

		// Returns the minimum dot product of the box with a given axis.
		T dotMin(const Vec& axis) const
		{
			T dmin, dmax;
			getExtentsAlongAxis(axis, dmin, dmax);
			return dmin;
		}

		// Returns the maximum dot product of the box with a given axis.
		T dotMax(const Vec& axis) const
		{
			T dmin, dmax;
			getExtentsAlongAxis(axis, dmin, dmax);
			return dmax;
		}

		// Returns a translated box.
		box translate(const Vec& v) const { return box(min + v, max + v); }

		// Returns a box grown by vector v (expanding symmetrically in both directions).
		box grow(const Vec& v) const { return box(min - v, max + v); }

		// Returns a box grown by a scalar value.
		box grow(T v) const { return box(min - Vec(v), max + Vec(v)); }

		// Returns a box with its bounds rounded.
		box round() const { return box(glm::round(min), glm::round(max)); }

		// Intersection operator.
		box operator & (const box& other) const { return box(glm::max(min, other.min), glm::min(max, other.max)); }
		box& operator &= (const box& other) { *this = *this & other; return *this; }

		// Union operator (smallest box that contains both).
		box operator | (const box& other) const { return box(glm::min(min, other.min), glm::max(max, other.max)); }
		box& operator |= (const box& other) { *this = *this | other; return *this; }

		// Expand the box to include a point.
		box operator | (const Vec& v) const { return box(glm::min(min, v), glm::max(max, v)); }
		box& operator |= (const Vec& v) { *this = *this | v; return *this; }

		// --- Equality Operators ---
		bool operator == (const box& other) const { return glm::all(glm::equal(min, other.min)) && glm::all(glm::equal(max, other.max)); }
		bool operator != (const box& other) const { return !(*this == other); }
	};

	using box2 = box<vec2>;
	using box3 = box<vec3>;

	inline box3 ComputeBox3(const glm::vec3& pos, const glm::quat& rot, const glm::vec3& scale)
	{
		box3 aabb;

		glm::vec3 localCorners[8] = {
			{-0.5f, -0.5f, -0.5f},
			{ 0.5f, -0.5f, -0.5f},
			{-0.5f,  0.5f, -0.5f},
			{ 0.5f,  0.5f, -0.5f},
			{-0.5f, -0.5f,  0.5f},
			{ 0.5f, -0.5f,  0.5f},
			{-0.5f,  0.5f,  0.5f},
			{ 0.5f,  0.5f,  0.5f}
		};

		glm::vec3 worldCorners[8];
		for (int i = 0; i < 8; i++) {
			worldCorners[i] = pos + (rot * (localCorners[i] * scale));
		}

		aabb.min = worldCorners[0];
		aabb.max = worldCorners[0];

		for (int i = 1; i < 8; i++)
		{
			aabb.min = glm::min(aabb.min, worldCorners[i]);
			aabb.max = glm::max(aabb.max, worldCorners[i]);
		}

		return aabb;
	}

	template <typename MatType, typename BoxType>
	BoxType ConvertBoxToWorldSpace(const MatType& worldTransform, const BoxType& box) 
	{
		using VecType = decltype(box.min);
		constexpr int Dim = VecType::length();

		std::array<VecType, (Dim == 2 ? 4 : 8)> corners{};

		if constexpr (Dim == 2) {
			corners = {
				box.min,
				VecType{ box.max.x, box.min.y },
				VecType{ box.min.x, box.max.y },
				box.max
			};
		}
		else if constexpr (Dim == 3) {
			corners = {
				box.min,
				VecType{ box.max.x, box.min.y, box.min.z },
				VecType{ box.min.x, box.max.y, box.min.z },
				VecType{ box.max.x, box.max.y, box.min.z },
				VecType{ box.min.x, box.min.y, box.max.z },
				VecType{ box.max.x, box.min.y, box.max.z },
				VecType{ box.min.x, box.max.y, box.max.z },
				box.max
			};
		}

		VecType worldMin(std::numeric_limits<typename VecType::value_type>::max());
		VecType worldMax(std::numeric_limits<typename VecType::value_type>::lowest());

		for (const auto& corner : corners) {
			auto transformedPoint4 = worldTransform * glm::vec<4, typename VecType::value_type>(corner, 1.0f);
			VecType transformedCorner;

			if constexpr (Dim == 2) {
				transformedCorner = VecType(transformedPoint4.x, transformedPoint4.y);
			}
			else if constexpr (Dim == 3) {
				transformedCorner = VecType(transformedPoint4.x, transformedPoint4.y, transformedPoint4.z);
			}

			worldMin = glm::min(worldMin, transformedCorner);
			worldMax = glm::max(worldMax, transformedCorner);
		}

		return BoxType{ worldMin, worldMax };
	}

	inline glm::mat3 CreateMat3(const glm::vec2 position, float rotation, const glm::vec2 scale)
	{
		float tx = position.x;
		float ty = position.y;
		float sx = scale.x;
		float sy = scale.y;
		float c = cos(rotation);
		float s = sin(rotation);

		return glm::mat3(
			sx * c, sx * s, tx,
			-sy * s, sy * c, ty,
			0.0f, 0.0f, 1.0f
		);
	};

	inline glm::mat4 CreateMat4(const glm::vec3 position, quat rotation, const glm::vec3 scale)
	{
		glm::mat4 transform(1.0f);

		// Set translation directly
		transform[3][0] = position.x;
		transform[3][1] = position.y;
		transform[3][2] = position.z;

		glm::mat4 rotationMatrix = glm::toMat4(rotation);

		// Set scaling directly
		transform[0][0] = scale.x;
		transform[1][1] = scale.y;
		transform[2][2] = scale.z;

		// Combine transformation in-place
		transform *= rotationMatrix;

		return transform;
	}

	template<int n> uint vectorToSnorm8(const vec<n, float>& v); 
	template<int n> vec<n, float> snorm8ToVector(uint v);

	template<>
	inline uint vectorToSnorm8(const float2& v)
	{
		float scale = 127.0f / sqrtf(v.x * v.x + v.y * v.y);
		int x = int(v.x * scale);
		int y = int(v.y * scale);
		return (x & 0xff) | ((y & 0xff) << 8);
	}

	template<>
	inline uint vectorToSnorm8(const float3& v)
	{
		float scale = 127.0f / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
		int x = int(v.x * scale);
		int y = int(v.y * scale);
		int z = int(v.z * scale);
		return (x & 0xff) | ((y & 0xff) << 8) | ((z & 0xff) << 16);
	}

	template<>
	inline uint vectorToSnorm8(const float4& v)
	{
		float scale = 127.0f / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
		int x = int(v.x * scale);
		int y = int(v.y * scale);
		int z = int(v.z * scale);
		int w = int(v.w * scale);
		return (x & 0xff) | ((y & 0xff) << 8) | ((z & 0xff) << 16) | ((w & 0xff) << 24);
	}
	
	template<>
	inline float2 snorm8ToVector(uint v)
	{
		float x = static_cast<signed char>(v & 0xff);
		float y = static_cast<signed char>((v >> 8) & 0xff);
		return max(float2(x, y) / 127.0f, float2(-1.f));
	}

	template<>
	inline float3 snorm8ToVector(uint v)
	{
		float x = static_cast<signed char>(v & 0xff);
		float y = static_cast<signed char>((v >> 8) & 0xff);
		float z = static_cast<signed char>((v >> 16) & 0xff);
		return max(float3(x, y, z) / 127.0f, float3(-1.f));
	}

	template<>
	inline float4 snorm8ToVector(uint v)
	{
		float x = static_cast<signed char>(v & 0xff);
		float y = static_cast<signed char>((v >> 8) & 0xff);
		float z = static_cast<signed char>((v >> 16) & 0xff);
		float w = static_cast<signed char>((v >> 24) & 0xff);
		return max(float4(x, y, z, w) / 127.0f, float4(-1.f));
	}
}