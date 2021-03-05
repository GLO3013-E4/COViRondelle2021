import PlannedTrajectory from '@/components/trajectories/PlannedTrajectory.vue';
import Vuex from 'vuex';
import { CoordinateFactory } from '@/factories/CoordinateFactory';

export default {
  title: 'components/trajectories/PlannedTrajectory',
  component: PlannedTrajectory,
};

const Template = (args: any) => ({
  components: { PlannedTrajectory },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<planned-trajectory/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  tableImage: '/stub_table_image.jpg',
  plannedTrajectory: CoordinateFactory.make(4),
};

export const TrajectoryToYellowPuck = Template.bind({}) as any;
TrajectoryToYellowPuck.args = {
  tableImage: '/stub_table_image1.jpg',
  plannedTrajectory: [
    { x: 1037.5, y: 512.5 },
    { x: 1037.5, y: 537.5 },
    { x: 1012.5, y: 537.5 },
    { x: 1012.5, y: 562.5 },
    { x: 987.5, y: 562.5 },
    { x: 987.5, y: 587.5 },
    { x: 962.5, y: 587.5 },
    { x: 962.5, y: 612.5 },
    { x: 937.5, y: 612.5 },
    { x: 912.5, y: 612.5 },
    { x: 887.5, y: 612.5 },
    { x: 862.5, y: 612.5 },
    { x: 837.5, y: 612.5 },
    { x: 812.5, y: 612.5 },
    { x: 787.5, y: 612.5 },
    { x: 787.5, y: 587.5 },
    { x: 762.5, y: 587.5 },
    { x: 762.5, y: 562.5 },
    { x: 737.5, y: 562.5 },
    { x: 737.5, y: 537.5 },
    { x: 712.5, y: 537.5 },
    { x: 712.5, y: 512.5 },
    { x: 687.5, y: 512.5 },
    { x: 687.5, y: 487.5 },
    { x: 662.5, y: 487.5 },
    { x: 662.5, y: 462.5 },
    { x: 637.5, y: 462.5 },
    { x: 637.5, y: 437.5 },
    { x: 637.5, y: 412.5 },
    { x: 612.5, y: 412.5 },
    { x: 587.5, y: 412.5 },
    { x: 562.5, y: 412.5 },
    { x: 537.5, y: 412.5 },
  ],
};
