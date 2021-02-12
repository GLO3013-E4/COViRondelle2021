import useVuetify from '@/hooks/useVuetify';
import RealTrajectory from '@/components/trajectories/RealTrajectory.vue';

const wrapper = useVuetify(RealTrajectory);

describe('When mounting real trajectory', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
