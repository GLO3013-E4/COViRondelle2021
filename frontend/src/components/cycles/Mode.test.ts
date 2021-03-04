import Mode from '@/components/cycles/Mode.vue';
import { createLocalVue, shallowMount } from '@vue/test-utils';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import Vuex from 'vuex';

const mockStore = (cycleReady:boolean, cycleStarted:boolean) => new Vuex.Store({
    state: {
      cycleReady:cycleReady,
      cycleStarted: cycleStarted
    },
  });

describe('When mounting Mode component', () => {
  const wrapper = wrapWithVuetifyAndStore(Mode);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given booting state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = mockStore(false,false)

  describe('When mounting Mode', () => {
    const wrapper = shallowMount(Mode, { store, localVue });

    it('Should be booting mode', () => {
      const mode = wrapper.findComponent({ ref: 'mode' });

      expect(mode.exists()).toBe(true);
      expect(mode.text()).toBe("Booting");
    });
  });
});

describe('Given waiting state', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
  
    const store = mockStore(true,false);
  
    describe('When mounting Mode', () => {
      const wrapper = shallowMount(Mode, { store, localVue });
  
      it('Should be booting mode', () => {
        const mode = wrapper.findComponent({ ref: 'mode' });
  
        expect(mode.exists()).toBe(true);
        expect(mode.text()).toBe("Waiting");
      });
    });
  });

  describe('Given started state', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
  
    const store = mockStore(true,true);
  
    describe('When mounting Mode', () => {
      const wrapper = shallowMount(Mode, { store, localVue });
  
      it('Should be started mode', () => {
        const mode = wrapper.findComponent({ ref: 'mode' });
  
        expect(mode.exists()).toBe(true);
        expect(mode.text()).toBe("Started");
      });
    });
  });
