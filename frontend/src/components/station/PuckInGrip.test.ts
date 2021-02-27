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

describe('Given state', () => {
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
      const switchComponent = wrapper.findComponent({ ref: 'switch' });

      expect(switchComponent.exists()).toBe(true);
      expect(store.state.puckInGrip).toBe(true);
    });

    it('Should not change state when clicked on', () => {
      const switchComponent = wrapper.findComponent({ ref: 'switch' });
      switchComponent.trigger('checked');
      expect(store.state.puckInGrip).toBe(true);
    });
  });
});
