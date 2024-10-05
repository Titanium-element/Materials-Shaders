
struct Helpers
{
    float randShape (float seed)
    {
        return frac(sin(seed) * 34567.8901);
    }
    float3 permute(float3 x) 
    { 
        return fmod(((x*34.0)+1.0)*x, 289.0); 
    }
    float3 fade(float3 t) 
    { 
        return t*t*t*(t*(t*6.0-15.0)+10.0); 
    }
    float3 rotate(float3 p, float angleX, float angleY, float angleZ)
    {
        float3x3 rotX = float3x3(
            1, 0, 0,
            0, cos(angleX), -sin(angleX),
            0, sin(angleX), cos(angleX)
        );

        float3x3 rotY = float3x3(
            cos(angleY), 0, sin(angleY),
            0, 1, 0,
            -sin(angleY), 0, cos(angleY)
        );

        float3x3 rotZ = float3x3(
            cos(angleZ), -sin(angleZ), 0,
            sin(angleZ), cos(angleZ), 0,
            0, 0, 1
        );

        float3x3 rotationMatrix = mul(mul(rotZ, rotY), rotX); 
        return mul(p, rotationMatrix);
    }
    float DistributionGGX(float N, float H, float roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float NdotH = max(0.0, dot(N,H));
        float NdotH2 = NdotH * NdotH;

        float denom = (NdotH2 * (a2 - 1.0) + 1.0);

        return a2 / denom;
    }
    float GeometrySmith(float3 N, float3 V, float3 L, float roughness)
    {
        float NdotV = max(0.0, dot(N, V));
        float r = (roughness + 1.0);
        float k = (r * r) / 8.0;
        float nom = NdotV;
        float denom = NdotV * (1.0 - k) + k;
        float ggx2 = nom/denom;

        float NdotL = max(0.0, dot(N, L));
        nom = NdotL;
        denom = NdotL * (1.0 - k) + k;
        float ggx1 = nom/denom;

        return ggx1 * ggx2;
    }
    float3 FresnelSchlick(float cosTheta, float3 F0)
    {
        return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
    }
};
struct SDFElements
{
    Helpers helper;
    float perlinNoise(float3 p)
    {
        float3 Pi = floor(p);
        float3 Pf = p - Pi;
        Pi = fmod(Pi, 289.0);
        float3 f = helper.fade(Pf);
        
        float n000 = dot(helper.permute(Pi + float3(0.0, 0.0, 0.0)), Pf);
        float n001 = dot(helper.permute(Pi + float3(0.0, 0.0, 1.0)), Pf - float3(0.0, 0.0, 1.0));
        float n010 = dot(helper.permute(Pi + float3(0.0, 1.0, 0.0)), Pf - float3(0.0, 1.0, 0.0));
        float n100 = dot(helper.permute(Pi + float3(1.0, 0.0, 0.0)), Pf - float3(1.0, 0.0, 0.0));
        float n011 = dot(helper.permute(Pi + float3(0.0, 1.0, 1.0)), Pf - float3(0.0, 1.0, 1.0));
        float n101 = dot(helper.permute(Pi + float3(1.0, 0.0, 1.0)), Pf - float3(1.0, 0.0, 1.0));
        float n110 = dot(helper.permute(Pi + float3(1.0, 1.0, 0.0)), Pf - float3(1.0, 1.0, 0.0));
        float n111 = dot(helper.permute(Pi + float3(1.0, 1.0, 1.0)), Pf - float3(1.0, 1.0, 1.0));

        float3 n00 = lerp(n000, n100, f.x);
        float3 n01 = lerp(n001, n101, f.x);
        float3 n10 = lerp(n010, n110, f.x);
        float3 n11 = lerp(n011, n111, f.x);
        
        float3 n0 = lerp(n00, n10, f.y);
        float3 n1 = lerp(n01, n11, f.y);
        
        return lerp(n0, n1, f.z); 
    }
    float boxSDF(float3 p, float3 b)
    {
        float3 side = abs(p) - b;
        return length(max(side, 0.0)) + min(max(side.x, max(side.y, side.z)), 0.0);
    }
    float hexagonalPrismSDF (float3 p, float2 h)
    {
        const float3 k = float3(-0.8660254, 0.5, 0.57735);
        p = abs(p);
        p.xy -= 2.0 * min(dot(k.xy, p.xy), 0.0) * k.xy;
        float2 d = float2(length(p.xy - float2(clamp(p.x, -k.z * h.x, k.z * h.x), h.x)) * sign(p.y - h.x), p.z - h.y);
        return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
    }
    float sphereSDF(float3 p, float radius)
    {
        return length(p) - radius;
    }
    float subtractionSDF (float d1, float d2)
    {
        return max(-d1, d2);
    }
}; 
struct sdfShape
{
    SDFElements sdfElement;
    float resultedSDF (float3 p, float radius, float distortions, float distortionStrength, float boxAmount, float3 objPos)
    {
        const float PI2 = 6.283185;
        float step = PI2 / boxAmount ;

        float angleX = 0.0;
        float angleY = 0.0; 
        float angleZ = 0.0;

        float basicSphere = sdfElement.sphereSDF(p, radius);
        float result = basicSphere;
        for(int i = 0; i < boxAmount; i++)
        {
            float bias = clamp(sdfElement.helper.randShape((objPos.x + objPos.y + objPos.z) * i) , 0.0, radians(30.0));
            float angleX = step * i + bias;
            float angleY = step * i + bias; 
            float angleZ = step * i + bias;
            float biasT;
            if (bias < 0.25)
            {
                biasT = lerp(-90.0, -70.0, bias * 2.0);  // Map [0, 0.5) to [-90, -70]
            }
            else
            {
                biasT = lerp(70.0, 90.0, (bias - 0.5) * 2.0);  // Map [0.5, 1] to [70, 90]
            }
            float3 biasTransform = float3(biasT, 0.0, 0.0);
            float3 boxPos = sdfElement.helper.rotate(p, angleX, angleY, angleZ ) + biasTransform; 
            float box = sdfElement.boxSDF(boxPos, float3(50, 80, 80)); 
            result = sdfElement.subtractionSDF(box, result);
        
        }   
        return result;
    }
    
}; sdfShape sdf;

const float3 moss1 = float3(0.333, 0.62, 0.314);
const float3 moss2 = float3(0.74, 0.98, 0.22);
const float3 moss3 = float3(0.02, 0.15, 0.01) * 0.5 ;

const float treshold = 0.01;
const float boxes = 12.0;
const float3 sphereCenter = float3(0.0, 0.0, 0.0);
float epsilon = 0.001;
const float3 F0 = float3(0.04, 0.04, 0.04);

float3 rayOrigin = (viewDir - worldPos);
float3 rayDir =  normalize(viewDir);

float rand = sdf.sdfElement.helper.randShape(objPos.x + objPos.y + objPos.z) % 10;

float noiseScale = 0.2;       
float noiseStrength = lerp(0.01, 0.02, rand);
epsilon *= 1.0 + noiseStrength;

for (int i = 0; i < 100; i++) 
{
    float3 pos = (rayOrigin - sphereCenter);
    float noiseValue = 0.0;

    float distance = sdf.resultedSDF(pos , sphereRadius, noiseValue, noiseStrength, boxes, objPos);

    if (distance < treshold)
    {
        float3 N = normalize(float3(sdf.resultedSDF(float3(pos.x + epsilon, pos.y, pos.z), sphereRadius, noiseValue, noiseStrength, boxes, objPos) - sdf.resultedSDF(float3(pos.x - epsilon, pos.y, pos.z), sphereRadius, noiseValue, noiseStrength, boxes, objPos),
                                        sdf.resultedSDF(float3(pos.x, pos.y + epsilon, pos.z), sphereRadius, noiseValue, noiseStrength, boxes, objPos) - sdf.resultedSDF(float3(pos.x, pos.y - epsilon, pos.z), sphereRadius, noiseValue, noiseStrength, boxes, objPos),
                                        sdf.resultedSDF(float3(pos.x, pos.y, pos.z + epsilon), sphereRadius, noiseValue, noiseStrength, boxes, objPos) - sdf.resultedSDF(float3(pos.x, pos.y, pos.z - epsilon), sphereRadius, noiseValue, noiseStrength, boxes, objPos)));
        
        float3 normal = abs(N);
        normal *= normal * normal * normal;
        normal /= normal.x + normal.y + normal.z;
        pos *= 0.01;
        float4 colXZ = Texture2DSample(texObject, texObjectSampler, pos.xz * 0.5 + 0.5);
        float4 colYZ = Texture2DSample(texObject, texObjectSampler, pos.yz * 0.5 + 0.5);
        float4 colXY = Texture2DSample(texObject, texObjectSampler, pos.xy * 0.5 + 0.5);
        float4 col = colYZ * normal.x + colXZ * normal.y + colXY * normal.z;
        float4 diffuseTex = col;
        float3 albedo = pow(diffuseTex.rgb, float3(2.2, 2.2, 2.2));

        colXZ = Texture2DSample(texObjectAO, texObjectAOSampler, pos.xz * 0.5 + 0.5);
        colYZ = Texture2DSample(texObjectAO, texObjectAOSampler, pos.yz * 0.5 + 0.5);
        colXY = Texture2DSample(texObjectAO, texObjectAOSampler, pos.xy * 0.5 + 0.5);
        col = colYZ * normal.x + colXZ * normal.y + colXY * normal.z;
        float3 aoMap = col.rgb;

        colXZ = Texture2DSample(texObjectNormal, texObjectNormalSampler, pos.xz * 0.5 + 0.5);
        colYZ = Texture2DSample(texObjectNormal, texObjectNormalSampler, pos.yz * 0.5 + 0.5);
        colXY = Texture2DSample(texObjectNormal, texObjectNormalSampler, pos.xy * 0.5 + 0.5);
        col = colYZ * normal.x + colXZ * normal.y + colXY * normal.z;
        float3 normalSample = col.rgb;
        normalSample = normalize(normalSample * 2.0 - 1.0);

        float3 tangent = normalize(cross(float3(0.0, 1.0, 0.0), N)); 
        float3 bitangent = cross(N, tangent);
        float3x3 TBN = float3x3(tangent, bitangent, N);
        // Transform sampled normal from tangent space to world space
        N = normalize(mul(normalSample, TBN));
        //LightCalcs
        pos *= 0.1;
        float3 Lo = float3(0.0, 0.0, 0.0);
        float3 V = normalize (viewDir - pos);
        float3 L = normalize(lightPos - pos);
        float3 H = normalize(V + L);
        float dist = 10000.0;
        float attenuation = 1.0 / (dist * dist) ;
        float3 radiance = float3(1.0, 1.0, 1.0)  ;

        float roughness =  max(aoMap.r * 1.25, 0.1);

        float NDF = sdf.sdfElement.helper.DistributionGGX(N, H, roughness);
        float G = sdf.sdfElement.helper.GeometrySmith(N, V, L, roughness);
        float3 F = sdf.sdfElement.helper.FresnelSchlick(max(0.0, dot(H, V)), F0);

        float3 numerator = NDF * G * F ;
        float denominator = 4.0 * max(0.0, dot(N, V)) * max(0.0, dot(N, L)) + 0.0001;
        float3 specular = numerator / denominator ;

        float3 kS = F;
        float3 kD = float3(1.0, 1.0, 1.0) - kS;

        float NdotL = max(dot(N, L), 0.0);  

        Lo = (kD * albedo / 3.14 + specular) * radiance * NdotL;
        //

        float ao = aoMap.g;
        float3 ambient = float3(0.01, 0.01, 0.01) * albedo * ao;
        float3 colour = ambient + Lo;
        colour = pow(colour, float3(1.0/2.2, 1.0/2.2, 1.0/2.2));

        float noiseTex = aoMap.b;
        float cracks = diffuseTex.a;
        float amount = 1.0 - pow(noiseTex, lerp(0.0, 1.2, saturate(time / 100)));
        cracks = lerp(1.0, cracks, amount);
       
        float3 mossColour = lerp(moss1, moss2, noiseTex * noiseTex) * 0.2 ;
        mossColour = lerp(moss3, mossColour, noiseTex );

        Lo = (kD * mossColour / 3.14 + specular * 10) * radiance * NdotL;
        ao = aoMap.b;
        ambient = float3(0.1, 0.1, 0.1) * mossColour * ao;
        mossColour = ambient + Lo;

        opacityMask = 1.0;
        return lerp(colour, mossColour, amount) * pow(cracks, 4.0); 
    }

    rayOrigin += rayDir * distance;
}
opacityMask = 0.0f;
return float3(0.0, 0.0, 0.0);

