import PuckDeposit from '@/components/station/PuckDeposit.vue';
import { Step } from '@/types/step';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { shallowMount, createLocalVue } from '@vue/test-utils';
import Vuex from 'vuex';
import * as faker from 'faker';
import { ColorFactory } from '@/factories/ColorFactory';

const mockStore = (puckInGrip: boolean, currentStep: Step) => 
    new Vuex.Store({
    state: {
      puckColors: ColorFactory.get(3),
      puckInGrip,
      currentStep,
    }
  });

describe('When mounting PuckDeposit component', () => {
  const wrapper = wrapWithVuetifyAndStore(PuckDeposit);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given no puck released yet', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = mockStore(false, Step.CycleNotStarted);
  
  describe('When mounting PuckDeposit', () => {
    const wrapper = shallowMount(PuckDeposit, { store, localVue });

    it('Should not have any puck released', () => {
        const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

        expect(pucks.exists()).toBe(false);
    });
  });
});

describe('Given first puck ready to get released', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
    const store = mockStore(true, Step.ToFirstCornerAndReleaseFirstPuck);
  
    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should not have any puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(false);
          expect(pucks).toHaveLength(0);
      });
    });
  });

  describe('Given first puck just got released', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
  
    const store = mockStore(false, Step.ToFirstCornerAndReleaseFirstPuck);

    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should have first puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(1);
      });
    });
  });

  describe('Given in between first puck and second puck released', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
  
    const store = mockStore(faker.random.boolean(), Step.ToSecondPuckAndGrabSecondPuck);
  
    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should have first puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(1);
      });
    });
  });

  describe('Given second puck just released', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);

    const store = mockStore(false, Step.ToSecondCornerAndReleaseSecondPuck);

    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should have first and second puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(2);
      });
    });
  });

  describe('Given before third puck release', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);

    const store = mockStore(false, Step.ToThirdPuckAndGrabThirdPuck);

    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should have first and second puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(2);
      });
    });
  });

  describe('Given third puck just released', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);

    const store = mockStore(false, Step.ToThirdCornerAndReleaseThirdPuck);

    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should have all puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(3);
      });
    });
  });

  describe('Given all puck released after thirdReleased step', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
  
    const store = mockStore(faker.random.boolean(), Step.ToSquareCenter);
  
    describe('When mounting PuckDeposit', () => {
      const wrapper = shallowMount(PuckDeposit, { store, localVue });
  
      it('Should have all puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(3);
      });
    });
  });


