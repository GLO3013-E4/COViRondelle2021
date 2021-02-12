import wrapWithVuetify from '@/util/wrapWithVuetify';
import RealTrajectory from '@/components/trajectories/RealTrajectory.vue';

const wrapper = wrapWithVuetify(RealTrajectory);

describe('When mounting real trajectory', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
