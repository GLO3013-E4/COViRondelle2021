import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';

export default {
  title: 'components/cycles/ConsumptionInformation',
  component: ConsumptionInformation,
};

export const Default = () => ({
  components: { ConsumptionInformation },
  template: `<consumption-information/>`,
});
