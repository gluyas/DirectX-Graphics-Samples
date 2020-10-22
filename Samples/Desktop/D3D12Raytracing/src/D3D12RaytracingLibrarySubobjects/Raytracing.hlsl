//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#ifndef RAYTRACING_HLSL
#define RAYTRACING_HLSL

#define HLSL
#include "Random.hlsli"
#include "RaytracingHlslCompat.h"

// Subobjects definitions at library scope.
GlobalRootSignature MyGlobalRootSignature =
{
    "DescriptorTable( UAV( u0 ) ),"                        // Output texture
    "SRV( t0 ),"                                           // Acceleration structure
    "CBV( b0 ),"                                           // Scene constants
    "DescriptorTable( SRV( t1, numDescriptors = 2 ) )"     // Static index and vertex buffers.
};

LocalRootSignature MyLocalRootSignature =
{
    "RootConstants( num32BitConstants = 1, b1 )"           // Cube constants
};

TriangleHitGroup LambertHitGroup =
{
    "",                    // AnyHit
    "LambertClosestHit",   // ClosestHit
};

SubobjectToExportsAssociation  LambertLocalRootSignatureAssociation =
{
    "MyLocalRootSignature",  // subobject name
    "LambertHitGroup"        // export association
};

TriangleHitGroup LightHitGroup =
{
    "",                  // AnyHit
    "LightClosestHit",   // ClosestHit
};

SubobjectToExportsAssociation  LightLocalRootSignatureAssociation =
{
    "MyLocalRootSignature",  // subobject name
    "LightHitGroup"          // export association
};

ProceduralPrimitiveHitGroup DielectricSphereHitGroup =
{
    "",
    "DielectricSphereClosestHit",
    "SphereIntersection"
};

SubobjectToExportsAssociation  DielectricSphereLocalRootSignatureAssociation =
{
    "MyLocalRootSignature",    // subobject name
    "DielectricSphereHitGroup" // export association
};

RaytracingShaderConfig  MyShaderConfig =
{
    32, // max payload size
    8   // max attribute size
};

typedef BuiltInTriangleIntersectionAttributes TriangleIntersectionAttributes;

struct SphereIntersectionAttributes {
    float Sign; // negative if the ray is exiting the sphere
};

struct RayPayload
{
    float3 Color;
    float  T;
    float3 Scatter; // TODO: material id?
    uint   RngSeed; // TODO: flags?
};

RaytracingPipelineConfig MyPipelineConfig =
{
    1 // max trace recursion depth
};

RaytracingAccelerationStructure Scene : register(t0, space0);
RWTexture2D<float4> RenderTarget : register(u0);
ByteAddressBuffer Indices : register(t1, space0);
StructuredBuffer<Vertex> Vertices : register(t2, space0);

ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);
ConstantBuffer<GeometryConstantBuffer> l_geometryCB : register(b1);

// Load three 16 bit indices from a byte addressed buffer.
uint3 Load3x16BitIndices(uint offsetBytes)
{
    uint3 indices;

    // ByteAdressBuffer loads must be aligned at a 4 byte boundary.
    // Since we need to read three 16 bit indices: { 0, 1, 2 }
    // aligned at a 4 byte boundary as: { 0 1 } { 2 0 } { 1 2 } { 0 1 } ...
    // we will load 8 bytes (~ 4 indices { a b | c d }) to handle two possible index triplet layouts,
    // based on first index's offsetBytes being aligned at the 4 byte boundary or not:
    //  Aligned:     { 0 1 | 2 - }
    //  Not aligned: { - 0 | 1 2 }
    const uint dwordAlignedOffset = offsetBytes & ~3;
    const uint2 four16BitIndices = Indices.Load2(dwordAlignedOffset);

    // Aligned: { 0 1 | 2 - } => retrieve first three 16bit indices
    if (dwordAlignedOffset == offsetBytes)
    {
        indices.x = four16BitIndices.x & 0xffff;
        indices.y = (four16BitIndices.x >> 16) & 0xffff;
        indices.z = four16BitIndices.y & 0xffff;
    }
    else // Not aligned: { - 0 | 1 2 } => retrieve last three 16bit indices
    {
        indices.x = (four16BitIndices.x >> 16) & 0xffff;
        indices.y = four16BitIndices.y & 0xffff;
        indices.z = (four16BitIndices.y >> 16) & 0xffff;
    }

    return indices;
}

// Retrieve hit world position.
float3 HitWorldPosition()
{
    return WorldRayOrigin() + RayTCurrent() * WorldRayDirection();
}

// Retrieve attribute at a hit position interpolated from vertex attributes using the hit's barycentrics.
float3 HitAttribute(float3 vertexAttribute[3], BuiltInTriangleIntersectionAttributes attr)
{
    return vertexAttribute[0] +
        attr.barycentrics.x * (vertexAttribute[1] - vertexAttribute[0]) +
        attr.barycentrics.y * (vertexAttribute[2] - vertexAttribute[0]);
}

inline Vertex HitInterpolatedVertexAttributes(TriangleIntersectionAttributes attr) {
    // Get the base index of the triangle's first 16 bit index.
    const uint INDEX_SIZE = 2;
    uint baseIndex = (l_geometryCB.bufferIndexOffset + 3*PrimitiveIndex())*INDEX_SIZE;

    // Load up 3 16 bit indices for the triangle.
    const uint3 indices = Load3x16BitIndices(baseIndex);

    float3 normals[3] = {
        Vertices[indices[0]].normal,
        Vertices[indices[1]].normal,
        Vertices[indices[2]].normal
    };
    float3 colors[3] = {
        Vertices[indices[0]].color,
        Vertices[indices[1]].color,
        Vertices[indices[2]].color
    };

    Vertex hit;
    hit.position = HitWorldPosition();
    hit.normal   = HitAttribute(normals, attr);
    hit.color    = HitAttribute(colors,  attr);
    return hit;
}

// Generate a ray in world space for a camera pixel corresponding to an index from the dispatched 2D grid.
inline void GenerateCameraRay(inout uint seed, uint2 index, float spread, out float3 origin, out float3 direction)
{
    float2 xy = index + 0.5f + RandomInsideUnitCircle(seed)*spread; // center in the middle of the pixel.
    float2 screenPos = xy / DispatchRaysDimensions().xy * 2.0 - 1.0;

    // Invert Y for DirectX-style coordinates.
    screenPos.y = -screenPos.y;

    // Unproject the pixel coordinate into a ray.
    float4 world = mul(float4(screenPos, 0, 1), g_sceneCB.projectionToWorld);

    world.xyz /= world.w;
    origin = g_sceneCB.cameraPosition.xyz;
    direction = normalize(world.xyz - origin);
}

[shader("raygeneration")]
void MyRaygenShader()
{
    RayPayload payload;
    payload.RngSeed = Hash(DispatchRaysIndex().xy);

    RayDesc ray;
    ray.TMin = 0.0001;
    ray.TMax = 10000.0;

    float3 pixelColor = 0;

    const uint SAMPLES = 256;
    const uint BOUNCES = 6;
    for (uint s = 0; s < SAMPLES; s++) {
        GenerateCameraRay(payload.RngSeed, DispatchRaysIndex().xy, 0.5*min(1, s), ray.Origin, ray.Direction);

        float3 rayColor = 1;
        for (uint b = 0; b < BOUNCES; b++) {
            TraceRay(Scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);

            rayColor *= payload.Color;
            if (!any(payload.Scatter)) {
                if (s == 0 && b == 0 && isinf(payload.T)) {
                    // terminates outer loop if first ray misses geometry (bad for antialiasing)
                    rayColor *= SAMPLES;
                    s = SAMPLES;
                }
                break;
            }

            ray.Origin   += payload.T * ray.Direction;
            ray.Direction = payload.Scatter;
        }
        pixelColor += rayColor;
    }
    pixelColor /= SAMPLES;

    // Write the raytraced color to the output texture.
    RenderTarget[DispatchRaysIndex().xy] = float4(pixelColor, 1.0);
}

[shader("closesthit")]
void LambertClosestHit(inout RayPayload payload, in TriangleIntersectionAttributes attr)
{
    Vertex hit = HitInterpolatedVertexAttributes(attr);

    // Generate scattered ray
    float3 scatter = RandomOnUnitSphere(payload.RngSeed);
    float  d       = dot(scatter, hit.normal);
    scatter -= min(0, 2*d) * hit.normal; // ensure scatter away from normal

    payload.T       = RayTCurrent();
    payload.Scatter = scatter;
    payload.Color   = abs(d) * hit.color;
}

[shader("closesthit")]
void LightClosestHit(inout RayPayload payload, in TriangleIntersectionAttributes attr)
{
    Vertex hit = HitInterpolatedVertexAttributes(attr);

    payload.T       = RayTCurrent();
    payload.Scatter = 0;
    payload.Color   = max(0, -dot(hit.normal, WorldRayDirection())) * hit.color;
}

struct DielectricSphere {
    float3 Position;
    float  Radius;
    float  RefractiveIndex;
    float3 Color;
};

#define SPHERE_HIT_ENTER 0x00
#define SPHERE_HIT_EXIT  0x01

inline DielectricSphere LoadDielectricSphere() {
    Vertex v = Vertices[l_geometryCB.bufferIndexOffset + PrimitiveIndex()];
    DielectricSphere sphere;
    sphere.Position        = v.position;
    sphere.Radius          = v.normal.x;
    sphere.RefractiveIndex = v.normal.y;
    sphere.Color           = v.color;
    return sphere;
}

[shader("intersection")]
void SphereIntersection()
{
    DielectricSphere sphere = LoadDielectricSphere();

    float3 rayOrigin    = WorldRayOrigin();
    float3 rayDirection = WorldRayDirection();

    float3 m = rayOrigin - sphere.Position;
    float  b = dot(m, rayDirection);
    float  c = dot(m, m) - sphere.Radius*sphere.Radius;

    if (c > 0 && b > 0) return;

    float d = b*b - c;
    if (d < 0) return;

    float e = sqrt(d);
    float t = -b - e;
    // TODO: respect face culling flags?
    SphereIntersectionAttributes attr;
    if (t >= 0) {
        // if ((RayFlags() & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES) == 0) return;
        attr.Sign = 1;
        ReportHit(t, SPHERE_HIT_ENTER, attr);
    } else {
        // if ((RayFlags() & RAY_FLAG_CULL_BACK_FACING_TRIANGLES)  == 0) return;
        t = -b + e;
        if (t >= 0) {
            attr.Sign = -1;
            ReportHit(t, SPHERE_HIT_EXIT, attr);
        }
    }
}

float Schlick(float cosine, float refractveIndex)
{
	float r0 = (1 - refractveIndex) / (1 + refractveIndex);
	r0 *= r0;
    float b = 1 - cosine;
	return r0 + (1 - r0) * b*b*b*b*b;
}

// TODO: make hit shaders independent of geometry type?
[shader("closesthit")]
void DielectricSphereClosestHit(inout RayPayload payload, in SphereIntersectionAttributes attr)
{
    DielectricSphere sphere = LoadDielectricSphere();

    float3 hit    = HitWorldPosition();
    float3 ray    = WorldRayDirection();
    float3 normal = (hit - sphere.Position) / sphere.Radius * attr.Sign;
    float  n      = sphere.RefractiveIndex;
    if (attr.Sign > 0) n = 1/n;
    float fresnel = Schlick(-dot(normal, ray), n);

    float3 scatter;
    if (Random01(payload.RngSeed) < fresnel) {
        scatter = reflect(ray, normal);
    } else {
        scatter = refract(ray, normal, n);
        if (!any(scatter)) {
            scatter = reflect(ray, normal);
        }
    }

    payload.T       = RayTCurrent();
    payload.Scatter = scatter;
    payload.Color   = 1; // TODO: absorption
}

[shader("miss")]
void MyMissShader(inout RayPayload payload)
{
    payload.Scatter = 0;
    payload.Color   = float3(0.0, 0.0, 0.2);
    payload.T       = INFINITY;
}

#endif // RAYTRACING_HLSL
