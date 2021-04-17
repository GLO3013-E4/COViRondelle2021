import ResetButton from "@/components/cycles/ResetButton.vue";

export default {
  title: 'components/cycles/ResetButton',
  component: ResetButton,
};

const Template = () => ({
  components: { ResetButton },
  template: '<ResetButton  v-bind="$props" />',
});

export const Basic = Template.bind({}) as any;
