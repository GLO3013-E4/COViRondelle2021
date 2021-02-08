import StationInformation from "@/components/station/StationInformation.vue";

export default {
  title: 'components/station/StationInformation',
  component: StationInformation,
};

export const Default = () => ({
  components: { StationInformation },
  template: `<PlannedTrajectory/>`
});