#ifdef GL_ES
precision mediump float;
#endif

// Phong related variables
uniform sampler2D uSampler;
uniform vec3 uKd;
uniform vec3 uKs;
uniform vec3 uLightPos;
uniform vec3 uCameraPos;
uniform vec3 uLightIntensity;

varying highp vec2 vTextureCoord;
varying highp vec3 vFragPos;
varying highp vec3 vNormal;

// Shadow map related variables
// #define NUM_SAMPLES 20
#define NUM_SAMPLES 64
#define BLOCKER_SEARCH_NUM_SAMPLES NUM_SAMPLES
#define PCF_NUM_SAMPLES NUM_SAMPLES
#define NUM_RINGS 10

#define EPS 1e-3
#define PI 3.141592653589793
#define PI2 6.283185307179586

// add by Gon laze
#define LIGHT_WIDTH 0.25   // default val in DirectionalLight.js: just use one-face-areaSize 

uniform sampler2D uShadowMap;
int initial = 0;

varying vec4 vPositionFromLight;

highp float rand_1to1(highp float x ) { 
  // -1 -1
  return fract(sin(x)*10000.0);
}

highp float rand_2to1(vec2 uv ) { 
  // 0 - 1
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract(sin(sn) * c);
}

float unpack(vec4 rgbaDepth) {
    const vec4 bitShift = vec4(1.0, 1.0/256.0, 1.0/(256.0*256.0), 1.0/(256.0*256.0*256.0));
    return dot(rgbaDepth, bitShift);
}

vec2 poissonDisk[NUM_SAMPLES];

void poissonDiskSamples( const in vec2 randomSeed ) {

  float ANGLE_STEP = PI2 * float( NUM_RINGS ) / float( NUM_SAMPLES );
  float INV_NUM_SAMPLES = 1.0 / float( NUM_SAMPLES );

  float angle = rand_2to1( randomSeed ) * PI2;
  float radius = INV_NUM_SAMPLES;
  float radiusStep = radius;

  for( int i = 0; i < NUM_SAMPLES; i ++ ) {
    poissonDisk[i] = vec2( cos( angle ), sin( angle ) ) * pow( radius, 0.75 );    // 0.75: empirical parameter for λ(space & time)
    radius += radiusStep;
    angle += ANGLE_STEP;
  }
}

void uniformDiskSamples( const in vec2 randomSeed ) {

  float randNum = rand_2to1(randomSeed);
  float sampleX = rand_1to1( randNum ) ;
  float sampleY = rand_1to1( sampleX ) ;

  float angle = sampleX * PI2;
  float radius = sqrt(sampleY);

  for( int i = 0; i < NUM_SAMPLES; i ++ ) {
    poissonDisk[i] = vec2( radius * cos(angle) , radius * sin(angle)  );

    sampleX = rand_1to1( sampleY ) ;
    sampleY = rand_1to1( sampleX ) ;

    angle = sampleX * PI2;
    radius = sqrt(sampleY);
  }
}

float findBlocker( sampler2D shadowMap,  vec2 uv, float zReceiver ) {

  float blockerDepth = unpack(texture2D(shadowMap, uv)); 
  // closer the blocker to the coords, smaller the blockSearch_region will be(scalene triangle)
  float blockSearch_region = LIGHT_WIDTH * ((zReceiver - blockerDepth) / zReceiver);
  float blockerAvgZ = 0.0;
  // getting avg blockerDepth around uv;
  // TODO: seems weight avg does not work well. Why?
  // float totalWeight = 0.0;
  // for (int i=0; i<BLOCKER_SEARCH_NUM_SAMPLES; i++)
  // {
  //   float sampleZ = unpack(texture2D(shadowMap, uv + poissonDisk[i] * blockSearch_region));
  //   if (sampleZ < zReceiver)    continue;
  //   else
  //   {
  //     float tmpW = max(1.0-dot(poissonDisk[i], poissonDisk[i]), 0.0);
  //     blockerAvgZ += sampleZ * tmpW;
  //     totalWeight += tmpW;  
  //   }   
  // }
  // if (totalWeight > 0.0)
  //   blockerAvgZ /= totalWeight;

  for (int i=0; i<BLOCKER_SEARCH_NUM_SAMPLES; i++)
    blockerAvgZ += unpack(texture2D(shadowMap, uv + poissonDisk[i] * blockSearch_region));
  blockerAvgZ /= float(BLOCKER_SEARCH_NUM_SAMPLES);

  return blockerAvgZ;
	// return 1.0;
}

float PCF(sampler2D shadowMap, vec4 coords) {

  // poisson disk
  poissonDiskSamples(coords.xy);
  // uniform disk
  // uniformDiskSamples(coords.xy);

  float shadowVal = 0.0;
  float filterSize = 1.0 * EPS;
  float bias = 0.5 * EPS;
  for (int i=0; i<PCF_NUM_SAMPLES; i++)
    shadowVal += ((unpack(texture2D(shadowMap, coords.xy + poissonDisk[i] * filterSize)) < coords.z-bias) ? 0.0 : 1.0);
  shadowVal /= float(PCF_NUM_SAMPLES);

  // you can see self-hidden happens(depends on your filterSize)
  // ! EPS bias should be set in calculation period(not the result!)
  return shadowVal;
  // return max(shadowVal-EPS, 0.0);
  // return 1.0;
}

float PCSS(sampler2D shadowMap, vec4 coords){

  // poisson disk
  poissonDiskSamples(coords.xy);
  // uniform disk
  // uniformDiskSamples(coords.xy);

  float shadowVal = 0.0;
  // * STEP 1: avgblocker depth
  float blockerAvgDepth = findBlocker(shadowMap, coords.xy, coords.z);
  // closer the blocker to the coords, smaller the filterSize will be(scalene triangle)
  // * STEP 2: penumbra size
  float filterSize = LIGHT_WIDTH * ((coords.z - blockerAvgDepth) / coords.z);
  // * STEP 3: filtering
  // same as PCF 
  float bias = 0.5 * EPS;
  for (int i=0; i<PCF_NUM_SAMPLES; i++)
    shadowVal += ((unpack(texture2D(shadowMap, coords.xy + poissonDisk[i] * filterSize)) < coords.z-bias) ? 0.0 : 1.0);
  shadowVal /= float(PCF_NUM_SAMPLES);

  // ! EPS bias should be set in calculation period(not the result!)
  return shadowVal;
  // return max(shadowVal-EPS, 0.0);
  // return 1.0;

}


float useShadowMap(sampler2D shadowMap, vec4 shadowCoord){

  // return ((unpack(texture2D(shadowMap, shadowCoord.xy)) < shadowCoord.z) ? 0.0 : 1.0);
  // * a simple bias can make it better!! (ref: https://zhuanlan.zhihu.com/p/668254886)
  float bias = 0.5 * EPS;
  return ((unpack(texture2D(shadowMap, shadowCoord.xy)) < shadowCoord.z-bias) ? 0.0 : 1.0);
  // return 1.0;
}

vec3 blinnPhong() {
  vec3 color = texture2D(uSampler, vTextureCoord).rgb;
  color = pow(color, vec3(2.2));

  vec3 ambient = 0.05 * color;

  vec3 lightDir = normalize(uLightPos);
  vec3 normal = normalize(vNormal);
  float diff = max(dot(lightDir, normal), 0.0);
  vec3 light_atten_coff =
      uLightIntensity / pow(length(uLightPos - vFragPos), 2.0);
  vec3 diffuse = diff * light_atten_coff * color;

  vec3 viewDir = normalize(uCameraPos - vFragPos);
  vec3 halfDir = normalize((lightDir + viewDir));
  float spec = pow(max(dot(halfDir, normal), 0.0), 32.0);
  vec3 specular = uKs * light_atten_coff * spec;

  vec3 radiance = (ambient + diffuse + specular);
  vec3 phongColor = pow(radiance, vec3(1.0 / 2.2));
  return phongColor;
}

void main(void) {

  vec3 shadowCoord = vPositionFromLight.xyz / vPositionFromLight.w;
  // from [-1.0, 1.0](ortho) to [0.0, 1.0](NDC)
  shadowCoord = shadowCoord * 0.5 + 0.5;

  /* add by Gon laze*/
  // * PCF initial step
  // ? seems this step should be set in each fragmentShader(the method that
  // ? each shader has a different disk randSeed seems better). Has been removed.
  // poisson disk
  // poissonDiskSamples(vec2(1.0, 1.0));
  // uniform disk
  // uniformDiskSamples(vec2(1.0, 1.0));

  float visibility;
  // visibility = useShadowMap(uShadowMap, vec4(shadowCoord, 1.0));
  // visibility = PCF(uShadowMap, vec4(shadowCoord, 1.0));
  visibility = PCSS(uShadowMap, vec4(shadowCoord, 1.0));

  vec3 phongColor = blinnPhong();

  gl_FragColor = vec4(phongColor * visibility, 1.0);
  // gl_FragColor = vec4(phongColor, 1.0);
}