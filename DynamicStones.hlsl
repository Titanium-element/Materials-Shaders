
struct sdfShapes 
{
    float sphere(float3 p, float radius)
    {
        return length(p) - radius;
    }

    float box(float3 p, float3 boxParams)
    {
        float3 q = abs(p) - boxParams;
        return length(max(0.0, q)) + min(max(q.x, max(q.y, q.z)), 0.0);
    }

    float smoothSubtract(float shape1, float shape2, float roundness)
    {
        float h = clamp(0.5 - 0.5 * (shape1 + shape2) / roundness, 0.0, 1.0);
        return lerp(shape1, -shape2, h) + roundness * h * (1.0 - h);
    }
    // Rotation matrix for rotation around a specific axis (e.g., around the Y-axis)
    float3 rotateY(float3 p, float angle)
    {
        float cosAngle = cos(angle);
        float sinAngle = sin(angle);
        float3 rotatedP;
        rotatedP.x = cosAngle * p.x - sinAngle * p.z;
        rotatedP.z = sinAngle * p.x + cosAngle * p.z;
        rotatedP.y = p.y; // Keep Y axis unchanged for rotation around Y-axis
        return rotatedP;
    }
};
sdfShapes sdf;

float3 rayOrigin = viewDir - worldPos;
float3 rayDir = (1) * viewDir;
float treshold = 0.01;
float angle = 3.14 / 4.0;  // Example: 45-degree rotation

// Raymarching loop
for (int i = 0; i < 100; i++) 
{
    // Apply rotation to the second box's position
    float3 rotatedRayOrigin = sdf.rotateY(rayOrigin - boxCenter2, angle);

    float resultedSDF = sdf.smoothSubtract(
        sdf.smoothSubtract(
            sdf.sphere(rayOrigin - sphereCenter, sphereRadius),
            sdf.box(rayOrigin - boxCenter, boxParams),
            0.5),
        sdf.box(rotatedRayOrigin, boxParams), 0.5
    );

    // Sample the distance field at the current point
    float distance = resultedSDF;

    // If we are close enough to the surface, return the color (hit the surface)
    if (distance < treshold)
    {
        return float4(1.0, 1.0, 1.0, 1.0);  // Hit the surface, return white color
    }

    rayOrigin += rayDir * distance;
}
return float4(0.0, 0.0, 0.0, 1.0);
