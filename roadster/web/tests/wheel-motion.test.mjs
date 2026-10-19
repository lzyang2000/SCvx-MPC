import test from 'node:test';
import assert from 'node:assert/strict';
import {rollingDistance} from '../src/wheel-motion.js';
test('wheel rotation follows body travel when the car lands facing the approach',()=>{const previous={p:[0,0,.62]},frame={phase:'SETTLED',p:[-1,0,.62],q:[0,0,0,1]};assert.equal(rollingDistance(frame,previous),1);assert.equal(rollingDistance({...frame,p:[1,0,.62]},previous),-1);assert.equal(rollingDistance({...frame,phase:'DESCENT'},previous),0);});
