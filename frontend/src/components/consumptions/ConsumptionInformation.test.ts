import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';
import wrapWithVuetify from '@/util/wrapWithVuetify';

const wrapper = wrapWithVuetify(ConsumptionInformation);

describe('When mounting consumption information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
