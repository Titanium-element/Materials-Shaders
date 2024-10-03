struct helpers
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
};
struct operations
{
    helpers helper;
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
        
        return lerp(n0, n1, f.z); // Final interpolation along z-axis
    }

    float2 sphereUV (float3 normal)
    {
        float PI = 3.1416 ;
        float u = 0.5 + atan2(normal.z, normal.x) / (2.0 * PI);
        float v = 0.5 - asin(normal.y) / PI;
        return float2(u, v);
    }
}; 
struct sdfShapes
{
    operations op;

    float distortedSphere(float3 p, float radius, float noiseValue, float noiseStrength)
    {
        float distortedRadius = radius + noiseValue * noiseStrength;
        return length(p) - distortedRadius;
    }
}; sdfShapes sdf;

float treshold = 0.01;
float epsilon = 0.0001;
float3 sphereCenter = float3(0.0, 0.0, 0.0);

float3 rayOrigin = (viewDir - worldPos);
float3 rayDir =  normalize(viewDir);
float3 lightDir = normalize(lightPos);

float seed = frac(objPos.x + objPos.y + objPos.z) % 10;
float scale = lerp(0.5, 1.0, seed);
// Noise parameters
float noiseScale = lerp(0.010, 0.020, seed);       
float noiseStrength = lerp(0.1, 0.5, seed); 


for (int i = 0; i < 100; i++) 
{
    float3 pos = (rayOrigin - sphereCenter) / scale;
    float noiseValue = sdf.op.perlinNoise(pos * noiseScale);

    float resultedSDF = sdf.distortedSphere(pos, sphereRadius, noiseValue, noiseStrength) * scale;
    float distance = resultedSDF;

    if (distance < treshold)
    {
        //float3 distPos = (distance - length(pos)) * (- rayDir) + epsilon *(- rayDir);
        float3 normal = normalize(pos);
        //float2 uv = sdf.op.sphereUV(normal);
        float2 uv = frac(worldPos.xy);

        float4 textureCol = Texture2DSample( texObject, texObjectSampler, uv);
        float3 aoMap = Texture2DSample(texObjectAO, texObjectAOSampler, uv).rgb;
        float3 normalSample =Texture2DSample(texObjectNormal, texObjectNormalSampler, uv).xyz;
        normalSample = normalize(normalSample * 2.0 - 1.0);

        float3 tangent = normalize(cross(float3(0.0, 1.0, 0.0), normal));  // Approximate tangent (adjust as needed)
        float3 bitangent = cross(normal, tangent);
        float3x3 TBN = float3x3(tangent, bitangent, normal);

        // Transform sampled normal from tangent space to world space
        float3 worldNormal = normalize(mul(normalSample, TBN));

        float diffuse = lerp(0.0, 1.0, max(0.0, dot(worldNormal, lightDir)));
        float roughness = aoMap.r;
        float ao = aoMap.g;
       
        opacityMask = 1.0f;
        return (textureCol.rgb *  lerp(0.0, 0.8, diffuse)) * ao * textureCol; 
        //return perturbedPos; 
    }

    rayOrigin += rayDir * distance;
}
opacityMask = 0.0f;
return float3(0.0, 0.0, 0.0);
