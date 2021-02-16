import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

const wrapper = wrapWithVuetifyAndStore(ConsumptionInformation);

describe('When mounting consumption information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
