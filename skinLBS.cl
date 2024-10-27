__kernel void skinLBS(
    __global float* finalPos,         // float3
    __global const float* initialPos, // float3
    __global const float* weights,    // float
    __global const uint* influences,  // uint
    __global const float4* matrices,  // mat4x3
    const uint positionCount
    )
{
    unsigned int positionId = get_global_id(0);
    if ( positionId >= positionCount )
    {
        return;
    }

    // FIXME: this constant should be sent from host
    const uint numInfluenceJoints = 2;

    // compute skinning matrix (4x3 matrix)
    float4 skinMat[3];
    for (uint c = 0; c < 3; c++) {
        skinMat[c] = (float4)(0.0f);
        for (uint wIdx = 0; wIdx < numInfluenceJoints; wIdx++) {
            const uint weightIdx = positionId * numInfluenceJoints + wIdx;
            skinMat[c] += weights[weightIdx] * matrices[influences[weightIdx] * 3 + c];
        }
    }

    // transform initial position by skinning matrix
    float4 initialPosition = (float4)(vload3( positionId , initialPos ), 1);
    float3 finalPosition;
    finalPosition.x = dot(skinMat[0], initialPosition);
    finalPosition.y = dot(skinMat[1], initialPosition);
    finalPosition.z = dot(skinMat[2], initialPosition);

    // store the result in the buffer
    vstore3( finalPosition , positionId , finalPos );
}
