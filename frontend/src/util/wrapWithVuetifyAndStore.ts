import Vuetify from 'vuetify';
import {createLocalVue, shallowMount, VueClass} from '@vue/test-utils';
import Vuex from 'vuex';
import VueI18n from 'vue-i18n';
import {defaultLocale, messages} from "@/i18n";

const wrapWithVuetifyAndStore = (component: VueClass<any>) => {
    const vuetify = new Vuetify();
    const localVue = createLocalVue();
    
    localVue.use(Vuex);
    localVue.use(VueI18n)
    
    const store = new Vuex.Store({});
    
    const i18n = new VueI18n({
       locale: defaultLocale,
       messages,
       silentTranslationWarn: true
    })

    return shallowMount(component, {vuetify, store, i18n, localVue});
};

export default wrapWithVuetifyAndStore;
