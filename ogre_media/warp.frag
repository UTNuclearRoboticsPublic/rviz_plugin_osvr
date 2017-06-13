#version 120

uniform sampler2D WarpTexture;

uniform vec2 LensCenter;
uniform vec2 Scale;
uniform vec2 ScaleIn;
uniform vec4 HmdWarpParam;

varying vec2 Texcoord;

// Scales input texture coordinates for distortion.
vec2 HmdWarp(vec2 texCoord)
{
	vec2 theta = (texCoord - LensCenter) * ScaleIn; // Scales texture coordinates to [-1, 1]
	float rSq = theta.x * theta.x + theta.y * theta.y;
	vec2 rvector= theta * (	HmdWarpParam.x +
							HmdWarpParam.y * rSq +
							HmdWarpParam.z * rSq * rSq +
							HmdWarpParam.w * rSq * rSq * rSq);
	return LensCenter + Scale * rvector;
}

vec2 Undistort(vec2 texCoord)
{
	vec2 theta = (texCoord - LensCenter) * ScaleIn; // Scales texture coordinates to [-1, 1]
	float rSq = theta.x * theta.x + theta.y * theta.y;
	float norm = sqrt(rSq);
	if (norm==0)
		norm=1;
	vec2 normVec = theta / norm;


	vec2 rvector= theta * (	0*pow(norm,0) +
							1*pow(norm,1) +
							-1.74*pow(norm,2) +
							5.15*pow(norm,3) +
							-1.27*pow(norm,4) +
							-2.23*pow(norm,5));

	return LensCenter + Scale * rvector;
}

void main(void)
{
// Radial distortion correction
//	vec2 tc = HmdWarp(Texcoord);
//	vec2 tc = Texcoord;
	vec2 tc = Undistort(Texcoord);
  gl_FragColor = texture2D(WarpTexture, tc);
}
