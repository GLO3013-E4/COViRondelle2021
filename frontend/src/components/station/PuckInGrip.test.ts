import PuckInGrip from '@/components/station/PuckInGrip.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { shallowMount, createLocalVue } from '@vue/test-utils';
import Vuex from 'vuex';

describe('When mounting PuckInGrip component', () => {
  const wrapper = wrapWithVuetifyAndStore(PuckInGrip);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given true state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
      puckInGrip: true,
    },
  });
  describe('When mounting PuckInGrip', () => {
    const wrapper = shallowMount(PuckInGrip, { store, localVue });

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(store.state.puckInGrip).toBe(true);
      expect(grip.text()).toBe('puck in grip');
    });
  });
});

describe('Given false state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
      puckInGrip: false,
    },
  });
  describe('When mounting PuckInGrip', () => {
    const wrapper = shallowMount(PuckInGrip, { store, localVue });

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(store.state.puckInGrip).toBe(false);
      expect(grip.text()).toBe('no puck');
    });
  });
});
