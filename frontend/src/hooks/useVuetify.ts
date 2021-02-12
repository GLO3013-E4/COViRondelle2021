import Vuetify from 'vuetify';
import { mount, VueClass } from '@vue/test-utils';

const useVuetify = (component: VueClass<any>) => {
  const vuetify = new Vuetify();
  return mount(component, { vuetify });
};

export default useVuetify;
