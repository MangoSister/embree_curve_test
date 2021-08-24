#include "curve_data.h"
#include <embree3/rtcore.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <limits>
#include <random>

void handle_embree_error(void *user_ptr, RTCError code, const char *str)
{
	if (code == RTC_ERROR_NONE)
		return;

	printf("Embree: ");
	switch (code) {
	case RTC_ERROR_UNKNOWN:
		printf("RTC_ERROR_UNKNOWN");
		break;
	case RTC_ERROR_INVALID_ARGUMENT:
		printf("RTC_ERROR_INVALID_ARGUMENT");
		break;
	case RTC_ERROR_INVALID_OPERATION:
		printf("RTC_ERROR_INVALID_OPERATION");
		break;
	case RTC_ERROR_OUT_OF_MEMORY:
		printf("RTC_ERROR_OUT_OF_MEMORY");
		break;
	case RTC_ERROR_UNSUPPORTED_CPU:
		printf("RTC_ERROR_UNSUPPORTED_CPU");
		break;
	case RTC_ERROR_CANCELLED:
		printf("RTC_ERROR_CANCELLED");
		break;
	default:
		printf("invalid error code");
		break;
	}
	if (str) {
		printf(" (");
		while (*str)
			putchar(*str++);
		printf(")\n");
	}
	exit(1);
}

inline void sample_uniform_sphere(float u1, float u2, float &dx, float &dy, float &dz)
{
	float z = 1.0f - 2.0f * u2;
	float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
	float phi = 3.1415926f * 2.0f;
	float sin_phi = std::sin(phi);
	float cos_phi = std::cos(phi);
	dx = r * cos_phi;
	dy = r * sin_phi;
	dz = z;
}

inline void sample_uniform_sphere_vol(float u1, float u2, float u3, float &x, float &y, float &z)
{
	float phi = u1 * 3.1415926f * 2.0f;
	float cos_theta = u2 * 2.0f - 1.0f;
	float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));
	float r = std::cbrt(u3); // cube root
	x = r * std::cos(phi) * sin_theta;
	y = r * std::sin(phi) * sin_theta;
	z = r * cos_theta;
}

int main(int argc, char *argv[])
{
	RTCDevice device = rtcNewDevice(nullptr);
	handle_embree_error(nullptr, rtcGetDeviceError(device), nullptr);
	rtcSetDeviceErrorFunction(device, handle_embree_error, nullptr);

	RTCScene rtcscene = rtcNewScene(device);
	RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE);
	rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, vertex_buffer, 0,
		sizeof(float[4]), num_vertices);
	rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, index_buffer, 0,
		sizeof(uint32_t), num_indices);
	rtcCommitGeometry(geom);
	rtcAttachGeometry(rtcscene, geom);
	rtcReleaseGeometry(geom);

	rtcCommitScene(rtcscene);

	std::random_device rd;  
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> uniform(0.0f, 1.0f);

	float closest_dist_max = 0.0f;
	float closest_dist_min = std::numeric_limits<float>::infinity();
	for (int i = 0; i < 100000; ++i) {
		RTCIntersectContext ctx;
		rtcInitIntersectContext(&ctx);
		RTCRayHit rayhit;

		sample_uniform_sphere(uniform(gen), uniform(gen), rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z);
		sample_uniform_sphere_vol(uniform(gen), uniform(gen), uniform(gen), rayhit.ray.org_x, rayhit.ray.org_y, rayhit.ray.org_z);
		rayhit.ray.org_x -= 2.0f * rayhit.ray.dir_x;
		rayhit.ray.org_y -= 2.0f * rayhit.ray.dir_y;
		rayhit.ray.org_z -= 2.0f * rayhit.ray.dir_z;

		rayhit.ray.tnear = 0.0f;
		rayhit.ray.time = 0.0f;
		rayhit.ray.tfar = std::numeric_limits<float>::infinity();
		rayhit.ray.mask = -1;
		rayhit.ray.id = 0;
		rayhit.ray.flags = 0;

		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
		rtcIntersect1(rtcscene, &ctx, &rayhit);
		if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
			//preview[y * image_width + x] = color3::Zero();
			float P[4];
			// embree does not calculate v or interpolate dpdv for linear curve...
			// also interpolate radius...
			rtcInterpolate1(rtcGetGeometry(rtcscene, rayhit.hit.geomID), rayhit.hit.primID, rayhit.hit.u, rayhit.hit.v,
				RTC_BUFFER_TYPE_VERTEX, 0, P, nullptr, nullptr, 4);

			float hit[3];
			hit[0] = rayhit.ray.org_x + rayhit.ray.dir_x * rayhit.ray.tfar;
			hit[1] = rayhit.ray.org_y + rayhit.ray.dir_y * rayhit.ray.tfar;
			hit[2] = rayhit.ray.org_z + rayhit.ray.dir_z * rayhit.ray.tfar;

			float dist = std::sqrt(
				(hit[0] - P[0]) * (hit[0] - P[0]) +
				(hit[1] - P[1]) * (hit[1] - P[1]) +
				(hit[2] - P[2]) * (hit[2] - P[2]));
			
			// printf("closest dist = %f, width = %f\n", dist, P[3]);
			
			closest_dist_max = std::max(closest_dist_max, dist);
			closest_dist_min = std::min(closest_dist_min, dist);
		}

	}
	printf("max closest dist = %f\n", closest_dist_max);
	printf("min closest dist = %f\n", closest_dist_min);

	rtcReleaseScene(rtcscene);
	rtcReleaseDevice(device);

    return 0;
}