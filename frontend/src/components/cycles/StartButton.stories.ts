import StartButton from '@/components/cycles/StartButton.vue';

export default {
  title: 'components/cycles/StartButton',
  component: StartButton,
};

const Template = () => ({
  components: { StartButton },
  template: '<StartButton  v-bind="$props" />',
});

export const Basic = Template.bind({}) as any;
