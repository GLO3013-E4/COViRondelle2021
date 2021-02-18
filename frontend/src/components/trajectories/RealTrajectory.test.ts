import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import RealTrajectory from '@/components/trajectories/RealTrajectory.vue';

const wrapper = wrapWithVuetifyAndStore(RealTrajectory);

describe('When mounting real trajectory', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
