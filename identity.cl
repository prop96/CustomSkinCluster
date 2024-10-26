__kernel void identity(
    __global float* finalPos,         // float3
    __global const float* initialPos, // float3
    const uint positionCount
    )
{
    unsigned int positionId = get_global_id(0);
    if ( positionId >= positionCount )
    {
        return;
    }

    float3 initialPosition = vload3( positionId , initialPos );
    
    // Perform some computation to get the final position.
    // ...
    float3 finalPosition = initialPosition;
    
    vstore3( finalPosition , positionId , finalPos );
}
