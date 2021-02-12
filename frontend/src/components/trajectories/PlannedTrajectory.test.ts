import PlannedTrajectory from '@/components/trajectories/PlannedTrajectory.vue';
import useVuetify from '@/hooks/useVuetify';

const wrapper = useVuetify(PlannedTrajectory);

describe('When mounting planned trajectory', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
