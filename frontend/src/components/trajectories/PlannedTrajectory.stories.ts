import PlannedTrajectory from '@/components/trajectories/PlannedTrajectory.vue';
import ControlPanel from '@/components/station/ControlPanel.vue';
import Vuex from 'vuex';
import { CornerFactory } from '@/factories/CornerFactory';
import { ColorFactory } from '@/factories/ColorFactory';

export default {
  title: 'components/trajectories/PlannedTrajectory',
  component: PlannedTrajectory,
};

// export const Default = () => ({
//   components: { PlannedTrajectory },
//   template: `<planned-trajectory/>`,
// });

const Template = (args: any) => ({
  components: { PlannedTrajectory },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<planned-trajectory/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  tableImage: '/test.jpg',
  plannedTrajectory: [
    {
      x: 20,
      y: 20,
    },
    {
      x: 40,
      y: 25,
    },
    {
      x: 60,
      y: 40,
    },
    {
      x: 80,
      y: 120,
    },
    {
      x: 120,
      y: 140,
    },
    {
      x: 200,
      y: 180,
    },
  ],
};
