import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';
import useVuetify from '@/hooks/useVuetify';

const wrapper = useVuetify(ConsumptionInformation);

describe('When mounting consumption information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
