Shader "ReconstructionTool/UI/RawImage Lens Distortion Correction"
{
    Properties
    {
        [PerRendererData] _MainTex ("Texture", 2D) = "white" {}
        _Color ("Tint", Color) = (1,1,1,1)
        _RadialK1 ("Radial K1", Float) = 0
        _RadialK2 ("Radial K2", Float) = 0
        _DistortionCenter ("Distortion Center", Vector) = (0.5,0.5,0,0)
        _CorrectionZoom ("Correction Zoom", Float) = 1
        _UvRect ("UV Rect", Vector) = (0,0,1,1)

        _StencilComp ("Stencil Comparison", Float) = 8
        _Stencil ("Stencil ID", Float) = 0
        _StencilOp ("Stencil Operation", Float) = 0
        _StencilWriteMask ("Stencil Write Mask", Float) = 255
        _StencilReadMask ("Stencil Read Mask", Float) = 255
        _ColorMask ("Color Mask", Float) = 15
        [Toggle(UNITY_UI_ALPHACLIP)] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0
    }

    SubShader
    {
        Tags
        {
            "Queue" = "Transparent"
            "IgnoreProjector" = "True"
            "RenderType" = "Transparent"
            "PreviewType" = "Plane"
            "CanUseSpriteAtlas" = "True"
        }

        Stencil
        {
            Ref [_Stencil]
            Comp [_StencilComp]
            Pass [_StencilOp]
            ReadMask [_StencilReadMask]
            WriteMask [_StencilWriteMask]
        }

        Cull Off
        Lighting Off
        ZWrite Off
        ZTest [unity_GUIZTestMode]
        Blend SrcAlpha OneMinusSrcAlpha
        ColorMask [_ColorMask]

        Pass
        {
            Name "Default"

            CGPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            #pragma target 2.0
            #pragma multi_compile_local _ UNITY_UI_CLIP_RECT
            #pragma multi_compile_local _ UNITY_UI_ALPHACLIP

            #include "UnityCG.cginc"
            #include "UnityUI.cginc"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float4 color : COLOR;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                fixed4 color : COLOR;
                float2 uv : TEXCOORD0;
                float4 positionOS : TEXCOORD1;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _MainTex;
            fixed4 _Color;
            fixed4 _TextureSampleAdd;
            float4 _MainTex_TexelSize;
            float4 _ClipRect;
            float4 _UvRect;
            float2 _DistortionCenter;
            float _RadialK1;
            float _RadialK2;
            float _CorrectionZoom;

            Varyings Vert(Attributes input)
            {
                Varyings output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);
                output.positionOS = input.positionOS;
                output.positionCS = UnityObjectToClipPos(input.positionOS);
                output.uv = input.uv;
                output.color = input.color * _Color;
                return output;
            }

            fixed4 Frag(Varyings input) : SV_Target
            {
                float2 uvSize = max(abs(_UvRect.zw), float2(0.000001, 0.000001));
                float2 uvDirection = step(float2(0.0, 0.0), _UvRect.zw) * 2.0 - 1.0;
                float2 safeUvSize = uvSize * uvDirection;
                float2 localUv = (input.uv - _UvRect.xy) / safeUvSize;
                float2 sourcePixels = max(
                    uvSize / max(abs(_MainTex_TexelSize.xy), float2(0.000001, 0.000001)),
                    float2(1.0, 1.0));
                float halfDiagonal = max(0.5 * length(sourcePixels), 0.000001);

                float2 radialPosition =
                    (localUv - _DistortionCenter) * sourcePixels /
                    (halfDiagonal * max(_CorrectionZoom, 0.000001));
                float radiusSquared = dot(radialPosition, radialPosition);
                float radialScale =
                    1.0 +
                    _RadialK1 * radiusSquared +
                    _RadialK2 * radiusSquared * radiusSquared;
                float2 sourceLocalUv =
                    _DistortionCenter +
                    radialPosition * radialScale * halfDiagonal / sourcePixels;
                float2 sourceUv = _UvRect.xy + sourceLocalUv * safeUvSize;

                float inside =
                    step(0.0, sourceLocalUv.x) *
                    step(sourceLocalUv.x, 1.0) *
                    step(0.0, sourceLocalUv.y) *
                    step(sourceLocalUv.y, 1.0);
                fixed4 color =
                    (tex2D(_MainTex, sourceUv) + _TextureSampleAdd) * input.color;
                color.a *= inside;

                #ifdef UNITY_UI_CLIP_RECT
                color.a *= UnityGet2DClipping(input.positionOS.xy, _ClipRect);
                #endif

                #ifdef UNITY_UI_ALPHACLIP
                clip(color.a - 0.001);
                #endif

                return color;
            }
            ENDCG
        }
    }
}
