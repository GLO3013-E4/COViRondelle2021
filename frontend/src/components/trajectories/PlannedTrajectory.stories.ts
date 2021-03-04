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
      x: 80,
      y: 120,
    },
    {
      x: 200,
      y: 180,
    },
    {
      x: 200,
      y: 230,
    },
    {
      x: 60,
      y: 230,
    }
  ],
};
