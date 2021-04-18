import ObjectivesInformation from '@/components/objectives/ObjectivesInformation.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

const wrapper = wrapWithVuetifyAndStore(ObjectivesInformation);

describe('When mounting objectives information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
