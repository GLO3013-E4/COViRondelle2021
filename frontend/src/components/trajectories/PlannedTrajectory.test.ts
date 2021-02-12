import PlannedTrajectory from '@/components/trajectories/PlannedTrajectory.vue';
import wrapWithVuetify from '@/util/wrapWithVuetify';

const wrapper = wrapWithVuetify(PlannedTrajectory);

describe('When mounting planned trajectory', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
