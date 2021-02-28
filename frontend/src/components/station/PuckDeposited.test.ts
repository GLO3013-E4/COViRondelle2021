import PuckDeposited from '@/components/station/PuckDeposited.vue';
import { StateFactory } from '@/factories/StateFactory';
import { state } from '@/store/state';
import { Color } from '@/types/color';
import { Step } from '@/types/step';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { shallowMount, createLocalVue } from '@vue/test-utils';
import Vuex from 'vuex';
import * as faker from 'faker';
import { ColorFactory } from '@/factories/ColorFactory';

describe('When mounting PuckDeposited component', () => {
  const wrapper = wrapWithVuetifyAndStore(PuckDeposited);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given no puck released yet', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
        puckColors: ColorFactory.get(3),
        puckInGrip: false,
        currentStep: Step.CycleNotStarted
    },
  });
  describe('When mounting PuckDeposited', () => {
    const wrapper = shallowMount(PuckDeposited, { store, localVue });

    it('Should not have any puck released', () => {
        const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

        expect(pucks.exists()).toBe(false);
    });
  });
});

describe('Given first puck ready to get released', () => {
    const localVue = createLocalVue();
    localVue.use(Vuex);
  
    const store = new Vuex.Store({
      state: {
          puckColors: ColorFactory.get(3),
          puckInGrip: true,
          currentStep: Step.ToFirstCornerAndReleaseFirstPuck
      },
    });
    describe('When mounting PuckDeposited', () => {
      const wrapper = shallowMount(PuckDeposited, { store, localVue });
  
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
  
    const store = new Vuex.Store({
      state: {
          puckColors: ColorFactory.get(3),
          puckInGrip: false,
          currentStep: Step.ToFirstCornerAndReleaseFirstPuck
      },
    });
    describe('When mounting PuckDeposited', () => {
      const wrapper = shallowMount(PuckDeposited, { store, localVue });
  
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
  
    const store = new Vuex.Store({
      state: {
          puckColors: ColorFactory.get(3),
          puckInGrip: faker.random.boolean,
          currentStep: Step.ToSecondPuck
      },
    });
    describe('When mounting PuckDeposited', () => {
      const wrapper = shallowMount(PuckDeposited, { store, localVue });
  
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
  
    const store = new Vuex.Store({
      state: {
          puckColors: ColorFactory.get(3),
          puckInGrip: false,
          currentStep: Step.ToSecondCornerAndReleaseSecondPuck
      },
    });
    describe('When mounting PuckDeposited', () => {
      const wrapper = shallowMount(PuckDeposited, { store, localVue });
  
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
  
    const store = new Vuex.Store({
      state: {
          puckColors: ColorFactory.get(3),
          puckInGrip: false,
          currentStep: Step.ToThirdCornerAndReleaseThirdPuck
      },
    });
    describe('When mounting PuckDeposited', () => {
      const wrapper = shallowMount(PuckDeposited, { store, localVue });
  
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
  
    const store = new Vuex.Store({
      state: {
          puckColors: ColorFactory.get(3),
          puckInGrip: faker.random.boolean,
          currentStep: Step.FinalRobotPlacementInGreenSquare
      },
    });
    describe('When mounting PuckDeposited', () => {
      const wrapper = shallowMount(PuckDeposited, { store, localVue });
  
      it('Should have all puck released', () => {
          const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });
  
          expect(pucks.exists()).toBe(true);
          expect(pucks).toHaveLength(3);
      });
    });
  });


