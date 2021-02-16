import PlannedTrajectory from '@/components/trajectories/PlannedTrajectory.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

const wrapper = wrapWithVuetifyAndStore(PlannedTrajectory);

describe('When mounting planned trajectory', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
