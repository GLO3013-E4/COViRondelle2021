import CycleInformation from '@/components/cycles/CycleInformation.vue';
import wrapWithVuetify from '@/util/wrapWithVuetify';

const wrapper = wrapWithVuetify(CycleInformation);

describe('When mounting cycle information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
