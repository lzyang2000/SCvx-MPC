import * as THREE from "three";

// A small ray-marched volume keeps the exhaust soft from every camera angle.
// Its animation uses simulation time, so pausing and exporting are repeatable.
export function createEnginePlume(radius, seed = 0) {
  const group = new THREE.Group();
  const uniforms = {
    cameraLocal: { value: new THREE.Vector3() },
    worldHeight: { value: new THREE.Vector4() },
    time: { value: 0 },
    seed: { value: seed },
    power: { value: 1 },
    expansion: { value: 0 },
  };
  const material = new THREE.ShaderMaterial({
    uniforms,
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    toneMapped: false,
    vertexShader: /* glsl */ `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      varying vec3 localPosition;
      void main() {
        localPosition = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        #include <logdepthbuf_vertex>
      }
    `,
    fragmentShader: /* glsl */ `
      precision highp float;
      #include <logdepthbuf_pars_fragment>
      varying vec3 localPosition;
      uniform vec3 cameraLocal;
      uniform vec4 worldHeight;
      uniform float time, seed, power, expansion;
      float hash(vec3 p) {
        p = fract(p * .3183099 + vec3(.17, .31, .73));
        p *= 17.0;
        return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
      }
      float noise(vec3 p) {
        vec3 cell = floor(p), f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        return mix(mix(mix(hash(cell), hash(cell + vec3(1,0,0)), f.x),
                       mix(hash(cell + vec3(0,1,0)), hash(cell + vec3(1,1,0)), f.x), f.y),
                   mix(mix(hash(cell + vec3(0,0,1)), hash(cell + vec3(1,0,1)), f.x),
                       mix(hash(cell + vec3(0,1,1)), hash(cell + vec3(1,1,1)), f.x), f.y), f.z);
      }
      void main() {
        #include <logdepthbuf_fragment>
        vec3 ray = normalize(localPosition - cameraLocal);
        // Preserve the sign for near-axis rays without dividing by zero.
        vec3 invRay = sign(ray + vec3(1e-7)) / max(abs(ray), vec3(1e-6));
        vec3 a = (vec3(-1,-1,-1) - cameraLocal) * invRay;
        vec3 b = (vec3(1,1,0) - cameraLocal) * invRay;
        vec3 nearFace = min(a,b), farFace = max(a,b);
        float entry = max(0.0, max(nearFace.x, max(nearFace.y, nearFace.z)));
        float exitAt = min(farFace.x, min(farFace.y, farFace.z));
        if (exitAt <= entry) discard;
        float stepSize = (exitAt - entry) / 32.0;
        vec3 light = vec3(0);
        float transmission = 1.0;
        for (int i = 0; i < 32; i++) {
          vec3 p = cameraLocal + ray * (entry + (float(i) + .5) * stepSize);
          if (dot(worldHeight, vec4(p,1)) < .12) continue;
          float z = -p.z;
          vec3 flow = vec3(p.xy * 8.0, z * 15.0 - time * 18.0 + seed);
          float n = noise(flow), fine = noise(flow * 2.1 + 4.7);
          vec2 bend = vec2(sin(z * 19.0 - time * 7.0 + seed),
                           cos(z * 16.0 - time * 9.0 + seed)) * .035 * z * z;
          // Expansion/compression near the bell forms a chain of shock cells.
          float cells = .5 + .5 * cos(z * 58.0);
          float width = (1.0 / (6.0 + 5.0 * expansion) + (.70 + .1 * expansion) * z)
                      * (1.0 - .17 * cells * exp(-z * 3.0));
          float radial = length(p.xy - bend) / width;
          float edge = 1.0 - smoothstep(.48 + .36 * n, 1.15, radial);
          float tail = pow(max(0.0, 1.0-z), .65)
                     * (1.0 - smoothstep(.65, 1.0, z))
                     * smoothstep(0.0, .025, z);
          float turbulent = mix(1.0, .35 + n * .85 + fine * .25, smoothstep(.12,.8,z));
          float core = exp(-radial * radial * 15.0) * exp(-z * 3.6);
          float diamond = pow(cells, 9.0) * exp(-z * 5.0);
          vec3 amber = mix(vec3(1.0,.12,.012), vec3(1.0,.54,.16), 1.0-z);
          vec3 emission = amber * (.9 + 1.1 * n)
                        + vec3(.38,.58,1.0) * exp(-z * 22.0) * .65
                        + vec3(1.0,.94,.78) * (core * 6.0 + diamond * core * 4.0);
          float density = edge * tail * turbulent * (1.4 + power);
          float alpha = 1.0 - exp(-density * stepSize * 6.0);
          light += transmission * emission * alpha;
          transmission *= 1.0-alpha;
        }
        gl_FragColor = vec4(light, (1.0-transmission) * .9);
        #include <colorspace_fragment>
      }
    `,
  });
  const geometry = new THREE.BoxGeometry(2, 2, 1);
  geometry.translate(0, 0, -.5);
  const volume = new THREE.Mesh(geometry, material);
  group.add(volume);
  const cameraPosition = new THREE.Vector3();
  volume.onBeforeRender = (_renderer, _scene, camera) => {
    camera.getWorldPosition(cameraPosition);
    uniforms.cameraLocal.value.copy(volume.worldToLocal(cameraPosition));
    const e = volume.matrixWorld.elements;
    uniforms.worldHeight.value.set(e[1], e[5], e[9], e[13]);
  };
  const glow = new THREE.PointLight(0xffb968, 0, 26, 2);
  glow.position.z = -.8;
  group.add(glow);
  group.visible = false;
  group.userData.update = (time, throttle, altitude) => {
    const power = THREE.MathUtils.clamp(throttle, 0, 1.2);
    const expansion = 1 - Math.exp(-Math.max(0, altitude) / 8500);
    uniforms.time.value = time % 1000;
    uniforms.power.value = power;
    uniforms.expansion.value = expansion;
    const width = radius * (6 + 5 * expansion);
    volume.scale.set(width, width, (6 + 19 * power) * (1 + .3 * expansion));
    // Warm light falls on the engine skirt, deployed legs and nearby pad.
    glow.intensity = 1000 * power;
  };
  return group;
}
