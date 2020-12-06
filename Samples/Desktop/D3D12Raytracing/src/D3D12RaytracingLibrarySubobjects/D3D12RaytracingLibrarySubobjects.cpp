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

#include "stdafx.h"

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"

#include "D3D12RaytracingLibrarySubobjects.h"
#include "DirectXRaytracingHelper.h"
#include "CompiledShaders\Raytracing.hlsl.h"

using namespace std;
using namespace DX;

const wchar_t* D3D12RaytracingLibrarySubobjects::c_lambertHitGroupName = L"LambertHitGroup";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_lightHitGroupName = L"LightHitGroup";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_dielectricSphereHitGroupName = L"DielectricSphereHitGroup";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_raygenShaderName = L"MyRaygenShader";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_lambertClosestHitShaderName = L"LambertClosestHit";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_lightClosestHitShaderName = L"LightClosestHit";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_dielectricSphereClosestHitShaderName = L"DielectricSphereClosestHit";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_sphereIntersectionShaderName = L"SphereIntersection";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_missShaderName = L"MyMissShader";

// Library subobject names
const wchar_t* D3D12RaytracingLibrarySubobjects::c_globalRootSignatureName = L"MyGlobalRootSignature";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_localRootSignatureName =  L"MyLocalRootSignature";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_lambertLocalRootSignatureAssociationName = L"LambertLocalRootSignatureAssociation";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_lightLocalRootSignatureAssociationName = L"LightLocalRootSignatureAssociation";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_dielectricSphereLocalRootSignatureAssociationName = L"DielectricSphereLocalRootSignatureAssociation";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_shaderConfigName = L"MyShaderConfig";
const wchar_t* D3D12RaytracingLibrarySubobjects::c_pipelineConfigName = L"MyPipelineConfig";

D3D12RaytracingLibrarySubobjects::D3D12RaytracingLibrarySubobjects(UINT width, UINT height, std::wstring name) :
    DXSample(width, height, name),
    m_raytracingOutputResourceUAVDescriptorHeapIndex(UINT_MAX),
    m_curRotationAngleRad(0.0f),
    m_isDxrSupported(false)
{
    UpdateForSizeChange(width, height);
}

void D3D12RaytracingLibrarySubobjects::EnableDirectXRaytracing(IDXGIAdapter1* adapter)
{
    m_isDxrSupported = IsDirectXRaytracingSupported(adapter);
    ThrowIfFalse(m_isDxrSupported, L"DirectX Raytracing is not supported by your GPU and driver.\n\n");
}

void D3D12RaytracingLibrarySubobjects::OnInit()
{
    m_deviceResources = std::make_unique<DeviceResources>(
        DXGI_FORMAT_R8G8B8A8_UNORM,
        DXGI_FORMAT_UNKNOWN,
        FrameCount,
        D3D_FEATURE_LEVEL_11_0,
        // Sample shows handling of use cases with tearing support, which is OS dependent and has been supported since TH2.
        // Since the Library Subobjects requires 19h1 OS update, we don't need to handle non-tearing cases.
        DeviceResources::c_RequireTearingSupport,
        m_adapterIDoverride
        );
    m_deviceResources->RegisterDeviceNotify(this);
    m_deviceResources->SetWindow(Win32Application::GetHwnd(), m_width, m_height);
    m_deviceResources->InitializeDXGIAdapter();
    EnableDirectXRaytracing(m_deviceResources->GetAdapter());

    m_deviceResources->CreateDeviceResources();
    m_deviceResources->CreateWindowSizeDependentResources();

    InitializeScene();

    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(Win32Application::GetHwnd());
    ImGui_ImplDX12_Init(m_deviceResources->GetD3DDevice(), FrameCount,
        DXGI_FORMAT_R8G8B8A8_UNORM, m_descriptorHeap.Get(),
        CD3DX12_CPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetCPUDescriptorHandleForHeapStart(), (UINT)3, m_descriptorSize),
        CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), (UINT)3, m_descriptorSize));

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);
}

// Update camera matrices passed into the shader.
void D3D12RaytracingLibrarySubobjects::UpdateCameraMatrices()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    m_sceneCB[frameIndex].cameraPosition = m_eye;
    float fovAngleY = 45.0f;
    XMMATRIX view = XMMatrixLookAtLH(m_eye, m_at, m_up);
    XMMATRIX proj = XMMatrixPerspectiveFovLH(XMConvertToRadians(fovAngleY), m_aspectRatio, 1.0f, 125.0f);
    XMMATRIX viewProj = view * proj;

    m_sceneCB[frameIndex].projectionToWorld = XMMatrixInverse(nullptr, viewProj);
}

// Initialize scene rendering parameters.
void D3D12RaytracingLibrarySubobjects::InitializeScene()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    // Setup camera.
    {
        // Initialize the view and projection inverse matrices.
        m_eye = { 0.0f, 0.0f, -5.0f, 1.0f };
        m_at = { 0.0f, 0.0f, 0.0f, 1.0f };
        XMVECTOR right = { 1.0f, 0.0f, 0.0f, 0.0f };

        XMVECTOR direction = XMVector4Normalize(m_at - m_eye);
        m_up = XMVector3Normalize(XMVector3Cross(direction, right));

        // Rotate camera around Y axis.
        XMMATRIX rotate = XMMatrixRotationY(XMConvertToRadians(45.0f));
        m_eye = XMVector3Transform(m_eye, rotate);
        m_up = XMVector3Transform(m_up, rotate);

        UpdateCameraMatrices();
    }

    // Setup lights.
    {
        // Initialize the lighting parameters.
        XMFLOAT4 lightPosition;
        XMFLOAT4 lightAmbientColor;
        XMFLOAT4 lightDiffuseColor;

        lightPosition = XMFLOAT4(0.0f, 1.8f, -3.0f, 0.0f);
        m_sceneCB[frameIndex].lightPosition = XMLoadFloat4(&lightPosition);

        lightAmbientColor = XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f);
        m_sceneCB[frameIndex].lightAmbientColor = XMLoadFloat4(&lightAmbientColor);

        lightDiffuseColor = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
        m_sceneCB[frameIndex].lightDiffuseColor = XMLoadFloat4(&lightDiffuseColor);
    }

    // Apply the initial values to all frames' buffer instances.
    for (auto& sceneCB : m_sceneCB)
    {
        sceneCB = m_sceneCB[frameIndex];
    }
}

// Create constant buffers.
void D3D12RaytracingLibrarySubobjects::CreateConstantBuffers()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    // Create the constant buffer memory and map the CPU and GPU addresses
    const D3D12_HEAP_PROPERTIES uploadHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);

    // Allocate one constant buffer per frame, since it gets updated every frame.
    size_t cbSize = frameCount * sizeof(AlignedSceneConstantBuffer);
    const D3D12_RESOURCE_DESC constantBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(cbSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &uploadHeapProperties,
        D3D12_HEAP_FLAG_NONE,
        &constantBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&m_perFrameConstants)));

    // Map the constant buffer and cache its heap pointers.
    // We don't unmap this until the app closes. Keeping buffer mapped for the lifetime of the resource is okay.
    CD3DX12_RANGE readRange(0, 0);        // We do not intend to read from this resource on the CPU.
    ThrowIfFailed(m_perFrameConstants->Map(0, nullptr, reinterpret_cast<void**>(&m_mappedConstantData)));
}

// Create resources that depend on the device.
void D3D12RaytracingLibrarySubobjects::CreateDeviceDependentResources()
{
    // Initialize raytracing pipeline.

    // Create raytracing interfaces: raytracing device and commandlist.
    CreateRaytracingInterfaces();

    // Create root signatures for the shaders.
    CreateRootSignatures();

    // Create a raytracing pipeline state object which defines the binding of shaders, state and resources to be used during raytracing.
    CreateRaytracingPipelineStateObject();

    // Create a heap for descriptors.
    CreateDescriptorHeap();

    // Build geometry to be used in the sample.
    BuildGeometry();

    // Build raytracing acceleration structures from the generated geometry.
    BuildAccelerationStructures();

    // Create constant buffers for the geometry and the scene.
    CreateConstantBuffers();

    // Build shader tables, which define shaders and their local root arguments.
    BuildShaderTables();

    // Create an output 2D texture to store the raytracing result to.
    CreateRaytracingOutputResource();
}

void D3D12RaytracingLibrarySubobjects::SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig)
{
    auto device = m_deviceResources->GetD3DDevice();
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error), error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&(*rootSig))));
}

void D3D12RaytracingLibrarySubobjects::CreateRootSignatures()
{
    auto device = m_deviceResources->GetD3DDevice();

    // A unique global root signature is defined in hlsl library g_pRaytracing. For such scenario we can create
    // compute root signature can directly from the library bytecode, using CreateRootSignature API.
    ThrowIfFailed(device->CreateRootSignature(1, g_pRaytracing, ARRAYSIZE(g_pRaytracing), IID_PPV_ARGS(&m_raytracingGlobalRootSignature)));
}

// Create raytracing device and command list.
void D3D12RaytracingLibrarySubobjects::CreateRaytracingInterfaces()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    ThrowIfFailed(device->QueryInterface(IID_PPV_ARGS(&m_dxrDevice)), L"Couldn't get DirectX Raytracing interface for the device.\n");

    // This fails for some reasom.
    // ThrowIfFailed(commandList->QueryInterface(IID_PPV_ARGS(&m_dxrCommandList)), L"Couldn't get DirectX Raytracing interface for the command list.\n");
    // Workaround
    m_dxrCommandList = reinterpret_cast<ID3D12GraphicsCommandList5*>(commandList);
}

// Create a raytracing pipeline state object (RTPSO).
// An RTPSO represents a full set of shaders reachable by a DispatchRays() call,
// with all configuration options resolved, such as local signatures and other state.
void D3D12RaytracingLibrarySubobjects::CreateRaytracingPipelineStateObject()
{
    // Create library and use the library subobject defined in the library in the RTPSO:
    // Subobjects need to be associated with DXIL shaders exports either by way of default or explicit associations.
    // Default association applies to every exported shader entrypoint that doesn't have any of the same type of subobject associated with it.
    // This simple sample utilizes default shader association except for local root signature subobject
    // which has an explicit association specified purely for demonstration purposes.
    //
    // Following subobjects are defined with the library itself. We can export and rename the subobjects from the
    // DXIL library in a similar way we export shaders.
    // 1 - Triangle hit group
    // 1 - Shader config
    // 2 - Local root signature and association
    // 1 - Global root signature
    // 1 - Pipeline config
    // 1 - Subobject to export association
    CD3DX12_STATE_OBJECT_DESC raytracingPipeline{ D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE };


    // DXIL library
    // This contains the shaders and their entrypoints for the state object.
    // Since shaders are not considered a subobject, they need to be passed in via DXIL library subobjects.
    auto lib = raytracingPipeline.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE((void *)g_pRaytracing, ARRAYSIZE(g_pRaytracing));
    lib->SetDXILLibrary(&libdxil);
    // Define which shader exports to surface from the library.
    // If no shader exports are defined for a DXIL library subobject, all shaders will be surfaced.
    // In this sample, this could be ommited for convenience since the sample uses all shaders in the library.
    {
        lib->DefineExport(c_raygenShaderName);
        lib->DefineExport(c_lambertClosestHitShaderName);
        lib->DefineExport(c_lightClosestHitShaderName);
        lib->DefineExport(c_dielectricSphereClosestHitShaderName);
        lib->DefineExport(c_sphereIntersectionShaderName);
        lib->DefineExport(c_missShaderName);
    }

    // Define which subobjects exports to use from the library.
    // If no exports are defined all subobjects are used.
    {
        lib->DefineExport(c_globalRootSignatureName);
        lib->DefineExport(c_localRootSignatureName);
        lib->DefineExport(c_lambertLocalRootSignatureAssociationName);
        lib->DefineExport(c_lightLocalRootSignatureAssociationName);
        lib->DefineExport(c_dielectricSphereLocalRootSignatureAssociationName);
        lib->DefineExport(c_shaderConfigName);
        lib->DefineExport(c_pipelineConfigName);
        lib->DefineExport(c_lambertHitGroupName);
        lib->DefineExport(c_lightHitGroupName);
        lib->DefineExport(c_dielectricSphereHitGroupName);
    }

#if _DEBUG
    PrintStateObjectDesc(raytracingPipeline);
#endif

    ThrowIfFailed(m_dxrDevice->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)), L"Couldn't create DirectX Raytracing state object.\n");
}

// Create 2D output texture for raytracing.
void D3D12RaytracingLibrarySubobjects::CreateRaytracingOutputResource()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    // Create the output resource. The dimensions and format should match the swap-chain.
    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(backbufferFormat, m_width, m_height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ThrowIfFailed(device->CreateCommittedResource(
        &defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_raytracingOutput)));
    NAME_D3D12_OBJECT(m_raytracingOutput);

    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
    m_raytracingOutputResourceUAVDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, m_raytracingOutputResourceUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
    UAVDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_raytracingOutput.Get(), nullptr, &UAVDesc, uavDescriptorHandle);
    m_raytracingOutputResourceUAVGpuDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), m_raytracingOutputResourceUAVDescriptorHeapIndex, m_descriptorSize);
}

void D3D12RaytracingLibrarySubobjects::CreateDescriptorHeap()
{
    auto device = m_deviceResources->GetD3DDevice();

    D3D12_DESCRIPTOR_HEAP_DESC descriptorHeapDesc = {};
    // Allocate a heap for 2 descriptors:
    // 2 - vertex and index buffer SRVs
    // 1 - raytracing output texture SRV
    descriptorHeapDesc.NumDescriptors = 5;
    descriptorHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    descriptorHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    descriptorHeapDesc.NodeMask = 0;
    device->CreateDescriptorHeap(&descriptorHeapDesc, IID_PPV_ARGS(&m_descriptorHeap));
    NAME_D3D12_OBJECT(m_descriptorHeap);

    m_descriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
}

// Build geometry used in the sample.
void D3D12RaytracingLibrarySubobjects::BuildGeometry()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Cube indices.
    Index indices[] =
    {
        // WALLS
        3,0,1,
        2,3,1,

        6,5,4,
        7,6,4,

        11,8,9,
        10,11,9,

        14,13,12,
        15,14,12,

        19,17,16,
        18,17,19,

        23,20,21,
        22,23,21,

        // LIGHT
        23,20,21,
        22,23,21,
    };

    // Cube vertices positions and corresponding triangle normals.
    Vertex vertices[] =
    {
        // TOP WALL
        { XMFLOAT3(-1.0f, 1.0f, -1.0f),  XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(1.0f, 1.0f, -1.0f),   XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(1.0f, 1.0f, 1.0f),    XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(-1.0f, 1.0f, 1.0f),   XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },

        // BOTTOM WALL
        { XMFLOAT3(-1.0f, -1.0f, -1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(1.0f, -1.0f, -1.0f),  XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(1.0f, -1.0f, 1.0f),   XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(-1.0f, -1.0f, 1.0f),  XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },

        // LEFT WALL
        { XMFLOAT3(-1.0f, -1.0f, 1.0f),  XMFLOAT3(1.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(-1.0f, -1.0f, -1.0f), XMFLOAT3(1.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(-1.0f, 1.0f, -1.0f),  XMFLOAT3(1.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(-1.0f, 1.0f, 1.0f),   XMFLOAT3(1.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },

        // RIGHT WALL
        { XMFLOAT3(1.0f, -1.0f, 1.0f),   XMFLOAT3(-1.0f, 0.0f, 0.0f), XMFLOAT3(1.0f, 0.0f, 0.0f) },
        { XMFLOAT3(1.0f, -1.0f, -1.0f),  XMFLOAT3(-1.0f, 0.0f, 0.0f), XMFLOAT3(1.0f, 0.0f, 0.0f) },
        { XMFLOAT3(1.0f, 1.0f, -1.0f),   XMFLOAT3(-1.0f, 0.0f, 0.0f), XMFLOAT3(1.0f, 0.0f, 0.0f) },
        { XMFLOAT3(1.0f, 1.0f, 1.0f),    XMFLOAT3(-1.0f, 0.0f, 0.0f), XMFLOAT3(1.0f, 0.0f, 0.0f) },

        // REAR WALL
        { XMFLOAT3(-1.0f, -1.0f, 1.0f),  XMFLOAT3(0.0f, 0.0f, -1.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(1.0f, -1.0f, 1.0f),   XMFLOAT3(0.0f, 0.0f, -1.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(1.0f, 1.0f, 1.0f),    XMFLOAT3(0.0f, 0.0f, -1.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(-1.0f, 1.0f, 1.0f),   XMFLOAT3(0.0f, 0.0f, -1.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },

        // TOP LIGHT
        // shader arguments specified per-vertex in BuildShaderTables
        // TODO: specify shader arguments at same time as geometry
        { XMFLOAT3(-0.5f, 0.9999f, -0.5f),  XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(10.0f, 10.0f, 10.0f) },
        { XMFLOAT3(0.5f, 0.9999f, -0.5f),   XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(10.0f, 10.0f, 10.0f) },
        { XMFLOAT3(0.5f, 0.9999f, 0.5f),    XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(10.0f, 10.0f, 10.0f) },
        { XMFLOAT3(-0.5f, 0.9999f, 0.5f),   XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(10.0f, 10.0f, 10.0f) },

        // REFRACTIVE SPHERE
        // TODO: including this in vertex buffer is really hacky!

        // 16-byte boundary: AABB (whole cube for now)
        // AABB stride set to 0 so this is used for all the following spheres
        { XMFLOAT3(-1.0f, -1.0f, -1.0f), XMFLOAT3(1.0f, 1.0f, 1.0f), XMFLOAT3(0.0f, 0.0f, 0.0f) },

        // implicit sphere: { float3 position; float radius; float refractiveIndex; float _pad20; float3 color; }
        { XMFLOAT3(0.25f, -0.4f, 0.25f), XMFLOAT3(0.4f, 1.5f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(-0.35f, 0.1f, -0.3f), XMFLOAT3(0.5f, 1.5f, 0.0f), XMFLOAT3(0.0f, 0.7f, 1.0f) },
    };

    AllocateUploadBuffer(device, indices, sizeof(indices), &m_indexBuffer.resource);
    AllocateUploadBuffer(device, vertices, sizeof(vertices), &m_vertexBuffer.resource);

    // Vertex buffer is passed to the shader along with index buffer as a descriptor table.
    // Vertex buffer descriptor must follow index buffer descriptor in the descriptor heap.
    UINT descriptorIndexIB = CreateBufferSRV(&m_indexBuffer, sizeof(indices)/4, 0);
    UINT descriptorIndexVB = CreateBufferSRV(&m_vertexBuffer, ARRAYSIZE(vertices), sizeof(vertices[0]));
    ThrowIfFalse(descriptorIndexVB == descriptorIndexIB + 1, L"Vertex Buffer descriptor index must follow that of Index Buffer descriptor index!");
}

// Build acceleration structures needed for raytracing.
void D3D12RaytracingLibrarySubobjects::BuildAccelerationStructures()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    auto commandQueue = m_deviceResources->GetCommandQueue();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();

    // Reset the command list for the acceleration structure construction.
    commandList->Reset(commandAllocator, nullptr);

    D3D12_RAYTRACING_GEOMETRY_DESC meshBaseGeometryDesc; // template for mesh instances
    meshBaseGeometryDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
    meshBaseGeometryDesc.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    meshBaseGeometryDesc.Triangles.IndexFormat = DXGI_FORMAT_R16_UINT;
    meshBaseGeometryDesc.Triangles.Transform3x4 = 0;
    meshBaseGeometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
    meshBaseGeometryDesc.Triangles.VertexBuffer.StartAddress = m_vertexBuffer.resource->GetGPUVirtualAddress();
    meshBaseGeometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
    meshBaseGeometryDesc.Triangles.VertexCount = 26;

    D3D12_RAYTRACING_GEOMETRY_DESC geometry[3];
    // walls
    geometry[0] = meshBaseGeometryDesc;
    geometry[0].Triangles.IndexBuffer = m_indexBuffer.resource->GetGPUVirtualAddress();
    geometry[0].Triangles.IndexCount = 30;

    // light
    geometry[1] = meshBaseGeometryDesc;
    geometry[1].Triangles.IndexBuffer = geometry[0].Triangles.IndexBuffer + geometry[0].Triangles.IndexCount*sizeof(Index);
    geometry[1].Triangles.IndexCount = 6;

    // sphere
    geometry[2].Type  = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
    geometry[2].Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    geometry[2].AABBs.AABBs.StartAddress = m_vertexBuffer.resource->GetGPUVirtualAddress() + 24*sizeof(Vertex);
    geometry[2].AABBs.AABBs.StrideInBytes = 0;
    geometry[2].AABBs.AABBCount = 2;

    // Get required sizes for an acceleration structure.
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC meshBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS &meshInputs = meshBuildDesc.Inputs;
    meshInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    meshInputs.Flags = buildFlags;
    meshInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    meshInputs.NumDescs = 2;
    meshInputs.pGeometryDescs = geometry + 0;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC aabbBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS &aabbInputs = aabbBuildDesc.Inputs;
    aabbInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    aabbInputs.Flags = buildFlags;
    aabbInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    aabbInputs.NumDescs = 1;
    aabbInputs.pGeometryDescs = geometry + 2;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS &topLevelInputs = topLevelBuildDesc.Inputs;
    topLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    topLevelInputs.Flags = buildFlags;
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.NumDescs = 2;
    topLevelInputs.InstanceDescs = NULL;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO topLevelPrebuildInfo;
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);

    ThrowIfFalse(topLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO meshPrebuildInfo = {};
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO aabbPrebuildInfo = {};

    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&meshInputs, &meshPrebuildInfo);
    ThrowIfFalse(meshPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&aabbInputs, &aabbPrebuildInfo);
    ThrowIfFalse(aabbPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    ComPtr<ID3D12Resource> scratchResource;
    AllocateUAVBuffer(device, max(topLevelPrebuildInfo.ScratchDataSizeInBytes, meshPrebuildInfo.ScratchDataSizeInBytes), &scratchResource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"ScratchResource");

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap equivalent).
    // Default heap is OK since the application doesn�t need CPU read/write access to them.
    // The resources that will contain acceleration structures must be created in the state D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
    // and must have resource flag D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both:
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using UAV barriers.
    {
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;

        AllocateUAVBuffer(device, meshPrebuildInfo.ResultDataMaxSizeInBytes, &m_meshAccelerationStructure, initialResourceState, L"MeshAccelerationStructure");
        AllocateUAVBuffer(device, aabbPrebuildInfo.ResultDataMaxSizeInBytes, &m_aabbAccelerationStructure, initialResourceState, L"AABBAccelerationStructure");
        AllocateUAVBuffer(device, topLevelPrebuildInfo.ResultDataMaxSizeInBytes, &m_topLevelAccelerationStructure, initialResourceState, L"TopLevelAccelerationStructure");
    }

    ComPtr<ID3D12Resource> instanceDescs;
    {
        D3D12_RAYTRACING_INSTANCE_DESC instanceDescsTemp[2];

        instanceDescsTemp[0].Transform[0][0] = instanceDescsTemp[0].Transform[1][1] = instanceDescsTemp[0].Transform[2][2] = 1;
        instanceDescsTemp[0].InstanceMask = 1;
        instanceDescsTemp[0].InstanceContributionToHitGroupIndex = 0;
        instanceDescsTemp[0].AccelerationStructure = m_meshAccelerationStructure->GetGPUVirtualAddress();

        instanceDescsTemp[1].Transform[0][0] = instanceDescsTemp[1].Transform[1][1] = instanceDescsTemp[1].Transform[2][2] = 1;
        instanceDescsTemp[1].InstanceMask = 1;
        instanceDescsTemp[1].InstanceContributionToHitGroupIndex = 2;
        instanceDescsTemp[1].AccelerationStructure = m_aabbAccelerationStructure->GetGPUVirtualAddress();

        AllocateUploadBuffer(device, &instanceDescsTemp, sizeof(instanceDescsTemp), &instanceDescs, L"InstanceDescs");
    }

    // mesh Bottom Level Acceleration Structure desc
    {
        meshBuildDesc.ScratchAccelerationStructureData = scratchResource->GetGPUVirtualAddress();
        meshBuildDesc.DestAccelerationStructureData = m_meshAccelerationStructure->GetGPUVirtualAddress();
    }

    // aabb Bottom Level Acceleration Structure desc
    {
        aabbBuildDesc.ScratchAccelerationStructureData = scratchResource->GetGPUVirtualAddress();
        aabbBuildDesc.DestAccelerationStructureData = m_aabbAccelerationStructure->GetGPUVirtualAddress();
    }

    // Top Level Acceleration Structure desc
    {
        topLevelBuildDesc.DestAccelerationStructureData = m_topLevelAccelerationStructure->GetGPUVirtualAddress();
        topLevelBuildDesc.ScratchAccelerationStructureData = scratchResource->GetGPUVirtualAddress();
        topLevelBuildDesc.Inputs.InstanceDescs = instanceDescs->GetGPUVirtualAddress();
    }

    auto BuildAccelerationStructure = [&](auto* raytracingCommandList)
    {
        raytracingCommandList->BuildRaytracingAccelerationStructure(&meshBuildDesc, 0, nullptr);
        commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(m_meshAccelerationStructure.Get()));
        raytracingCommandList->BuildRaytracingAccelerationStructure(&aabbBuildDesc, 0, nullptr);
        commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(m_aabbAccelerationStructure.Get()));
        raytracingCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);
    };

    BuildAccelerationStructure(m_dxrCommandList);

    // Kick off acceleration structure construction.
    m_deviceResources->ExecuteCommandList();

    // Wait for GPU to finish as the locally created temporary GPU resources will get released once we go out of scope.
    m_deviceResources->WaitForGpu();
}

// Build shader tables.
// This encapsulates all shader records - shaders and the arguments for their local root signatures.
void D3D12RaytracingLibrarySubobjects::BuildShaderTables()
{
    auto device = m_deviceResources->GetD3DDevice();

    void* rayGenShaderIdentifier;
    void* missShaderIdentifier;
    void* lambertHitGroupShaderIdentifier;
    void* lightHitGroupShaderIdentifier;
    void* dielectricSphereHitGroupShaderIdentifier;

    auto GetShaderIdentifiers = [&](auto* stateObjectProperties)
    {
        rayGenShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_raygenShaderName);
        missShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_missShaderName);
        lambertHitGroupShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_lambertHitGroupName);
        lightHitGroupShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_lightHitGroupName);
        dielectricSphereHitGroupShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_dielectricSphereHitGroupName);
    };

    // Get shader identifiers.
    UINT shaderIdentifierSize;
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_dxrStateObject.As(&stateObjectProperties));
        GetShaderIdentifiers(stateObjectProperties.Get());
        shaderIdentifierSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

    // Ray gen shader table
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIdentifierSize;
        ShaderTable rayGenShaderTable(device, numShaderRecords, shaderRecordSize, L"RayGenShaderTable");
        rayGenShaderTable.push_back(ShaderRecord(rayGenShaderIdentifier, shaderIdentifierSize));
        m_rayGenShaderTable = rayGenShaderTable.GetResource();
    }

    // Miss shader table
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIdentifierSize;
        ShaderTable missShaderTable(device, numShaderRecords, shaderRecordSize, L"MissShaderTable");
        missShaderTable.push_back(ShaderRecord(missShaderIdentifier, shaderIdentifierSize));
        m_missShaderTable = missShaderTable.GetResource();
    }

    // Hit group shader table
    {
        struct RootArguments {
            GeometryConstantBuffer cb;
        } rootArguments;

        UINT numShaderRecords = 3; // TODO: dynamic based on geometry description
        UINT shaderRecordSize = shaderIdentifierSize + sizeof(rootArguments);
        ShaderTable hitGroupShaderTable(device, numShaderRecords, shaderRecordSize, L"HitGroupShaderTable");
        // lambert shader
        {
            rootArguments.cb.bufferIndexOffset = 0; // offset into index buffer
            ShaderRecord lambertShaderRecord(lambertHitGroupShaderIdentifier, shaderIdentifierSize, &rootArguments, sizeof(rootArguments));
            hitGroupShaderTable.push_back(lambertShaderRecord);
        }
        // light shader
        {
            rootArguments.cb.bufferIndexOffset = 30; // offset into index buffer
            ShaderRecord lightShaderRecord(lightHitGroupShaderIdentifier, shaderIdentifierSize, &rootArguments, sizeof(rootArguments));
            hitGroupShaderTable.push_back(lightShaderRecord);
        }
        // dielectricSphere shader
        {
            rootArguments.cb.bufferIndexOffset = 25; // offset into vertex buffer
            ShaderRecord dielectricSphereShaderRecord(dielectricSphereHitGroupShaderIdentifier, shaderIdentifierSize, &rootArguments, sizeof(rootArguments));
            hitGroupShaderTable.push_back(dielectricSphereShaderRecord);
        }
        m_hitGroupShaderTable = hitGroupShaderTable.GetResource();
        m_hitGroupShaderTableStride = hitGroupShaderTable.GetShaderRecordSize();
    }
}

// Update frame-based values.
void D3D12RaytracingLibrarySubobjects::OnUpdate()
{
    // Start the Dear ImGui frame
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    static bool showDemoWindow = true;
    if (showDemoWindow) ImGui::ShowDemoWindow(&showDemoWindow);

    m_timer.Tick();
    CalculateFrameStats();
    float elapsedTime = static_cast<float>(m_timer.GetElapsedSeconds());
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();
    auto prevFrameIndex = m_deviceResources->GetPreviousFrameIndex();

    // Rotate the camera around Y axis.
    {
        float secondsToRotateAround = 24.0f;
        float angleToRotateBy = 360.0f * (elapsedTime / secondsToRotateAround);
        XMMATRIX rotate = XMMatrixRotationY(XMConvertToRadians(angleToRotateBy));
        m_eye = XMVector3Transform(m_eye, rotate);
        m_up = XMVector3Transform(m_up, rotate);
        m_at = XMVector3Transform(m_at, rotate);
        UpdateCameraMatrices();
    }

    // Rotate the second light around Y axis.
    {
        float secondsToRotateAround = 8.0f;
        float angleToRotateBy = -360.0f * (elapsedTime / secondsToRotateAround);
        XMMATRIX rotate = XMMatrixRotationY(XMConvertToRadians(angleToRotateBy));
        const XMVECTOR& prevLightPosition = m_sceneCB[prevFrameIndex].lightPosition;
        m_sceneCB[frameIndex].lightPosition = XMVector3Transform(prevLightPosition, rotate);
    }

    ImGui::EndFrame();
}

void D3D12RaytracingLibrarySubobjects::DoRaytracing()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    auto DispatchRays = [&](auto* commandList, auto* stateObject, auto* dispatchDesc)
    {
        // Since each shader table has only one shader record, the stride is same as the size.
        dispatchDesc->HitGroupTable.StartAddress = m_hitGroupShaderTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes = m_hitGroupShaderTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes = m_hitGroupShaderTableStride;
        dispatchDesc->MissShaderTable.StartAddress = m_missShaderTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes = m_missShaderTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes = dispatchDesc->MissShaderTable.SizeInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_rayGenShaderTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes = m_rayGenShaderTable->GetDesc().Width;
        dispatchDesc->Width = m_width;
        dispatchDesc->Height = m_height;
        dispatchDesc->Depth = 1;
        commandList->SetPipelineState1(stateObject);
        commandList->DispatchRays(dispatchDesc);
    };

    auto SetCommonPipelineState = [&](auto* descriptorSetCommandList)
    {
        descriptorSetCommandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
        // Set index and successive vertex buffer decriptor tables
        commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::VertexBuffersSlot, m_indexBuffer.gpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, m_raytracingOutputResourceUAVGpuDescriptor);
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Copy the updated scene constant buffer to GPU.
    memcpy(&m_mappedConstantData[frameIndex].constants, &m_sceneCB[frameIndex], sizeof(m_sceneCB[frameIndex]));
    auto cbGpuAddress = m_perFrameConstants->GetGPUVirtualAddress() + frameIndex * sizeof(m_mappedConstantData[0]);
    commandList->SetComputeRootConstantBufferView(GlobalRootSignatureParams::SceneConstantSlot, cbGpuAddress);

    // Bind the heaps, acceleration structure and dispatch rays.
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    {
        SetCommonPipelineState(commandList);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignatureParams::AccelerationStructureSlot, m_topLevelAccelerationStructure->GetGPUVirtualAddress());
        DispatchRays(m_dxrCommandList, m_dxrStateObject.Get(), &dispatchDesc);
    }
}

// Update the application state with the new resolution.
void D3D12RaytracingLibrarySubobjects::UpdateForSizeChange(UINT width, UINT height)
{
    DXSample::UpdateForSizeChange(width, height);
}

// Copy the raytracing output to the backbuffer.
void D3D12RaytracingLibrarySubobjects::CopyRaytracingOutputToBackbuffer()
{
    auto commandList= m_deviceResources->GetCommandList();
    auto renderTarget = m_deviceResources->GetRenderTarget();

    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    preCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
    preCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    commandList->ResourceBarrier(ARRAYSIZE(preCopyBarriers), preCopyBarriers);

    commandList->CopyResource(renderTarget, m_raytracingOutput.Get());

    D3D12_RESOURCE_BARRIER postCopyBarriers[2];
    postCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
    postCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->ResourceBarrier(ARRAYSIZE(postCopyBarriers), postCopyBarriers);
}

// Create resources that are dependent on the size of the main window.
void D3D12RaytracingLibrarySubobjects::CreateWindowSizeDependentResources()
{
    CreateRaytracingOutputResource();
    UpdateCameraMatrices();
}

// Release resources that are dependent on the size of the main window.
void D3D12RaytracingLibrarySubobjects::ReleaseWindowSizeDependentResources()
{
    m_raytracingOutput.Reset();
}

// Release all resources that depend on the device.
void D3D12RaytracingLibrarySubobjects::ReleaseDeviceDependentResources()
{
    m_raytracingGlobalRootSignature.Reset();

    m_dxrDevice.Reset();
    m_dxrStateObject.Reset();

    m_descriptorHeap.Reset();
    m_descriptorsAllocated = 0;
    m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;
    m_indexBuffer.resource.Reset();
    m_vertexBuffer.resource.Reset();
    m_perFrameConstants.Reset();
    m_rayGenShaderTable.Reset();
    m_missShaderTable.Reset();
    m_hitGroupShaderTable.Reset();

    m_meshAccelerationStructure.Reset();
    m_aabbAccelerationStructure.Reset();
    m_topLevelAccelerationStructure.Reset();

}

void D3D12RaytracingLibrarySubobjects::RecreateD3D()
{
    // Give GPU a chance to finish its execution in progress.
    try
    {
        m_deviceResources->WaitForGpu();
    }
    catch (HrException&)
    {
        // Do nothing, currently attached adapter is unresponsive.
    }
    m_deviceResources->HandleDeviceLost();
}

// Render the scene.
void D3D12RaytracingLibrarySubobjects::OnRender()
{
    if (!m_deviceResources->IsWindowVisible())
    {
        return;
    }

    m_deviceResources->Prepare();
    DoRaytracing();
    CopyRaytracingOutputToBackbuffer();

    /* render imgui */ {
        auto commandList   = m_deviceResources->GetCommandList();
        auto renderTarget  = m_deviceResources->GetRenderTarget();

        D3D12_RESOURCE_BARRIER barrier = {};
        barrier = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
        commandList->ResourceBarrier(1, &barrier);
        commandList->OMSetRenderTargets(1, &m_deviceResources->GetRenderTargetView(), FALSE, NULL);
        // NOTE: ensure ImGui descriptor heap is set
        ImGui::Render();
        ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), commandList);

        barrier = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
        commandList->ResourceBarrier(1, &barrier);
    }

    m_deviceResources->Present(D3D12_RESOURCE_STATE_PRESENT);
}

void D3D12RaytracingLibrarySubobjects::OnDestroy()
{
    // Let GPU finish before releasing D3D resources.
    m_deviceResources->WaitForGpu();

    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    OnDeviceLost();
}

// Release all device dependent resouces when a device is lost.
void D3D12RaytracingLibrarySubobjects::OnDeviceLost()
{
    ReleaseWindowSizeDependentResources();
    ReleaseDeviceDependentResources();
}

// Create all device dependent resources when a device is restored.
void D3D12RaytracingLibrarySubobjects::OnDeviceRestored()
{
    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();
}

// Compute the average frames per second and million rays per second.
void D3D12RaytracingLibrarySubobjects::CalculateFrameStats()
{
    static int frameCnt = 0;
    static double elapsedTime = 0.0f;
    double totalTime = m_timer.GetTotalSeconds();
    frameCnt++;

    // Compute averages over one second period.
    if ((totalTime - elapsedTime) >= 1.0f)
    {
        float diff = static_cast<float>(totalTime - elapsedTime);
        float fps = static_cast<float>(frameCnt) / diff; // Normalize to an exact second.

        frameCnt = 0;
        elapsedTime = totalTime;

        float MRaysPerSecond = (m_width * m_height * fps) / static_cast<float>(1e6);

        wstringstream windowText;

        windowText << setprecision(2) << fixed
            << L"    fps: " << fps << L"     ~Million Primary Rays/s: " << MRaysPerSecond
            << L"    GPU[" << m_deviceResources->GetAdapterID() << L"]: " << m_deviceResources->GetAdapterDescription();
        SetCustomWindowText(windowText.str().c_str());
    }
}

// Handle OnSizeChanged message event.
void D3D12RaytracingLibrarySubobjects::OnSizeChanged(UINT width, UINT height, bool minimized)
{
    if (!m_deviceResources->WindowSizeChanged(width, height, minimized))
    {
        return;
    }

    UpdateForSizeChange(width, height);

    ReleaseWindowSizeDependentResources();
    CreateWindowSizeDependentResources();
}

// Allocate a descriptor and return its index.
// If the passed descriptorIndexToUse is valid, it will be used instead of allocating a new one.
UINT D3D12RaytracingLibrarySubobjects::AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse)
{
    auto descriptorHeapCpuBase = m_descriptorHeap->GetCPUDescriptorHandleForHeapStart();
    if (descriptorIndexToUse >= m_descriptorHeap->GetDesc().NumDescriptors)
    {
        descriptorIndexToUse = m_descriptorsAllocated++;
    }
    *cpuDescriptor = CD3DX12_CPU_DESCRIPTOR_HANDLE(descriptorHeapCpuBase, descriptorIndexToUse, m_descriptorSize);
    return descriptorIndexToUse;
}

// Create SRV for a buffer.
UINT D3D12RaytracingLibrarySubobjects::CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize)
{
    auto device = m_deviceResources->GetD3DDevice();

    // SRV
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Buffer.NumElements = numElements;
    if (elementSize == 0)
    {
        srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_RAW;
        srvDesc.Buffer.StructureByteStride = 0;
    }
    else
    {
        srvDesc.Format = DXGI_FORMAT_UNKNOWN;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
        srvDesc.Buffer.StructureByteStride = elementSize;
    }
    UINT descriptorIndex = AllocateDescriptor(&buffer->cpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->cpuDescriptorHandle);
    auto x = m_descriptorHeap->GetGPUDescriptorHandleForHeapStart();
    buffer->gpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(x, descriptorIndex, m_descriptorSize);
    return descriptorIndex;
}
