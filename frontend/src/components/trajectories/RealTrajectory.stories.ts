import RealTrajectory from "@/components/trajectories/RealTrajectory.vue";

export default {
  title: 'components/trajectories/RealTrajectory',
  component: RealTrajectory,
};

export const Default = () => ({
  components: { RealTrajectory },
  template: `<real-trajectory/>`
});