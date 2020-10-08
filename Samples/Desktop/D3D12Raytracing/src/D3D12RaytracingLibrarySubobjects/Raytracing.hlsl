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
    "RootConstants( num32BitConstants = 4, b1 )"           // Cube constants
};

TriangleHitGroup MyHitGroup =
{
    "",                     // AnyHit
    "MyClosestHitShader",   // ClosestHit
};

SubobjectToExportsAssociation  MyLocalRootSignatureAssociation =
{
    "MyLocalRootSignature",  // subobject name
    "MyHitGroup"             // export association
};

RaytracingShaderConfig  MyShaderConfig =
{
    32, // max payload size
    8   // max attribute size
};

typedef BuiltInTriangleIntersectionAttributes MyAttributes;

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
ConstantBuffer<CubeConstantBuffer> g_cubeCB : register(b1);

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

// Generate a ray in world space for a camera pixel corresponding to an index from the dispatched 2D grid.
inline void GenerateCameraRay(inout uint seed, uint2 index, out float3 origin, out float3 direction)
{
    float2 xy = index + 0.5f + RandomInsideUnitCircle(seed)*0.5; // center in the middle of the pixel.
    float2 screenPos = xy / DispatchRaysDimensions().xy * 2.0 - 1.0;

    // Invert Y for DirectX-style coordinates.
    screenPos.y = -screenPos.y;

    // Unproject the pixel coordinate into a ray.
    float4 world = mul(float4(screenPos, 0, 1), g_sceneCB.projectionToWorld);

    world.xyz /= world.w;
    origin = g_sceneCB.cameraPosition.xyz;
    direction = normalize(world.xyz - origin);
}

// Diffuse lighting calculation.
float4 CalculateDiffuseLighting(float3 hitPosition, float3 normal)
{
    float3 pixelToLight = normalize(g_sceneCB.lightPosition.xyz - hitPosition);

    // Diffuse contribution.
    float fNDotL = max(0.0f, dot(pixelToLight, normal));

    return g_cubeCB.albedo * g_sceneCB.lightDiffuseColor * fNDotL;
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

    const uint SAMPLES = 32;
    const uint BOUNCES = 8;
    for (uint s = 0; s < SAMPLES; s++) {
        GenerateCameraRay(payload.RngSeed, DispatchRaysIndex().xy, ray.Origin, ray.Direction);

        float3 rayColor = 1;
        for (uint b = 0; b < BOUNCES; b++) {
            TraceRay(Scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);

            rayColor *= payload.Color;
            if (!any(payload.Scatter)) { break; }

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
void MyClosestHitShader(inout RayPayload payload, in MyAttributes attr)
{
    float3 hitPosition = HitWorldPosition();

    // Get the base index of the triangle's first 16 bit index.
    uint indexSizeInBytes = 2;
    uint indicesPerTriangle = 3;
    uint triangleIndexStride = indicesPerTriangle * indexSizeInBytes;
    uint baseIndex = PrimitiveIndex() * triangleIndexStride;

    // Load up 3 16 bit indices for the triangle.
    const uint3 indices = Load3x16BitIndices(baseIndex);

    // Retrieve corresponding vertex normals for the triangle vertices.
    float3 vertexNormals[3] = {
        Vertices[indices[0]].normal,
        Vertices[indices[1]].normal,
        Vertices[indices[2]].normal
    };

    // Compute the triangle's normal.
    // This is redundant and done for illustration purposes
    // as all the per-vertex normals are the same and match triangle's normal in this sample.
    float3 triangleNormal = HitAttribute(vertexNormals, attr);

    float4 diffuseColor = CalculateDiffuseLighting(hitPosition, triangleNormal);
    float4 color = g_sceneCB.lightAmbientColor + diffuseColor;

    // Generate scattered ray
    float3 scatter = RandomOnUnitSphere(payload.RngSeed);
    float  d       = dot(scatter, triangleNormal);
    scatter -= min(0, 2*d) * triangleNormal; // ensure scatter away from normal

    payload.T       = RayTCurrent();
    payload.Scatter = scatter;
    payload.Color   = abs(d)*float3(Vertices[indices[0]].color);
}

[shader("miss")]
void MyMissShader(inout RayPayload payload)
{
    payload.Scatter = 0;
    payload.Color   = 1.5;
    payload.T       = INFINITY;
}

#endif // RAYTRACING_HLSL
