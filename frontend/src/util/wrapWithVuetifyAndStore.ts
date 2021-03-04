import Vuetify from 'vuetify';
import { createLocalVue, shallowMount, VueClass } from '@vue/test-utils';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';

const wrapWithVuetifyAndStore = (component: VueClass<any>) => {
  const vuetify = new Vuetify();
  const localVue = createLocalVue();
  localVue.use(Vuex);
  const store = new Vuex.Store({
    state: StateFactory.make(),
  });

  return shallowMount(component, { vuetify, store, localVue });
};

export default wrapWithVuetifyAndStore;
