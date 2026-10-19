import {rotate,dot,sub} from './compute/math.js';
/** Signed travel along the car's own forward axis, including reverse heading. */
export function rollingDistance(frame,previous){
 if(!['APPROACH','RAMP','SETTLED'].includes(frame.phase))return 0;
 return dot(sub(frame.p,previous.p),rotate(frame.q,[1,0,0]));
}
