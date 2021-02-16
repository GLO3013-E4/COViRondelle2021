import CycleInformation from '@/components/cycles/CycleInformation.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

const wrapper = wrapWithVuetifyAndStore(CycleInformation);

describe('When mounting cycle information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
