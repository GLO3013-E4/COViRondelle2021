import PlannedTrajectory from "@/components/trajectories/PlannedTrajectory.vue";

export default {
  title: 'components/trajectories/PlannedTrajectory',
  component: PlannedTrajectory,
};

export const Default = () => ({
  components: { PlannedTrajectory },
  template: `<planned-trajectory/>`
});