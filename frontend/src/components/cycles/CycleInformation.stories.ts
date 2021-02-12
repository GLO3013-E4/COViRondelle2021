import CycleInformation from "@/components/cycles/CycleInformation.vue";

export default {
  title: "components/cycles/CycleInformation",
  component: CycleInformation,
};

export const Default = () => ({
  components: { CycleInformation },
  template: `<cycle-information/>`,
});
