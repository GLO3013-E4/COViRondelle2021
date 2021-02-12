import Vuetify from 'vuetify';
import { mount, VueClass } from '@vue/test-utils';

const wrapWithVuetify = (component: VueClass<any>) => {
  const vuetify = new Vuetify();
  return mount(component, { vuetify });
};

export default wrapWithVuetify;
