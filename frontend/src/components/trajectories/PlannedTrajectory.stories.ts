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
