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

#ifndef RANDOM_HLSLI
#define RANDOM_HLSLI

#include "Util.hlsli"

// Ref: http://www.reedbeta.com/blog/quick-and-easy-gpu-random-numbers-in-d3d11/

uint Hash(uint seed)
{
    // Thomas Wang hash
    // Ref: http://www.burtleburtle.net/bob/hash/integer.html
    seed = (seed ^ 61) ^ (seed >> 16);
    seed *= 9;
    seed = seed ^ (seed >> 4);
    seed *= 0x27d4eb2d;
    seed = seed ^ (seed >> 15);
    return seed;
}

uint Hash(uint2 seed) {
    return Hash(seed.y) + 31*Hash(seed.x);
}

uint Hash(uint3 seed) {
    return Hash(seed.z) + 31*Hash(seed.xy);
}

uint Hash(uint4 seed) {
    return Hash(seed.w) + 31*Hash(seed.xyz);
}

// Generate a random 32-bit integer
uint Random(inout uint seed)
{
    // Xorshift algorithm from George Marsaglia's paper.
    seed ^= (seed << 13);
    seed ^= (seed >> 17);
    seed ^= (seed << 5);
    return seed;
}

// Generate a random float in the range [0.0f, 1.0f)
float Random01(inout uint seed)
{
    return asfloat(0x3f800000 | Random(seed) >> 9) - 1.0;
}

// Generate a random float in the range (-1.0f, 1.0f)
float Random01Signed(inout uint seed)
{
    uint rand = Random(seed);
    return asfloat(asuint(asfloat(0x3f800000 | rand >> 9) - 1.0) | (rand & 0x80000000));
}

// Generate a random integer in the range [lower, upper]
uint Random(inout uint seed, uint lower, uint upper)
{
    return lower + uint(float(upper - lower + 1) * Random01(seed));
}

// Generate a random float in the range [lower, upper)
float Random(inout uint seed, float lower, float upper)
{
    return lower + (upper - lower)*Random01(seed);
}

float2 RandomInsideUnitCircle(inout uint seed)
{
    float2 unit;
    float  unit2;
    do {
        unit.x = Random01Signed(seed);
        unit.y = Random01Signed(seed);
        unit2 = dot(unit, unit);
    } while (unit2 > 1.0);
    return unit;
}

float2 RandomOnUnitCircle(inout uint seed)
{
    float theta = Random01(seed)*TAU;
    return float2(cos(theta), sin(theta));
}

float3 RandomInsideUnitSphere(inout uint seed)
{
    float3 unit;
    float  unit2;
    do {
        unit.x = Random01Signed(seed);
        unit.y = Random01Signed(seed);
        unit.z = Random01Signed(seed);
        unit2 = dot(unit, unit);
    } while (unit2 > 1.0);
    return unit;
}

float3 RandomOnUnitSphere(inout uint seed)
{
    float theta1 = Random01(seed)*TAU;
    float theta2 = Random01(seed)*TAU;
    float cos1   = cos(theta1);
    return float3(cos1*cos(theta2), cos1*sin(theta2), sin(theta1));
}

float3 RandomInsideHemisphere(inout uint seed, float3 normal) {
    float3 unit = RandomInsideUnitSphere(seed);
    return unit - min(0.0, 2.0*dot(normal, unit))*normal;
}

float3 RandomOnHemisphere(inout uint seed, float3 normal) {
    float3 unit = RandomOnUnitSphere(seed);
    return unit - min(0.0, 2.0*dot(normal, unit))*normal;
}

#endif // RANDOM_HLSLI
