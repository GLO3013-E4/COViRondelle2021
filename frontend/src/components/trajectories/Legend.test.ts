import Legend from '@/components/trajectories/Legend.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

describe('When mounting Legend', () => {

  const wrapper = wrapWithVuetifyAndStore(Legend);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});