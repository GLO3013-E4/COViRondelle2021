import Vue from 'vue';
import Vuetify from 'vuetify';
import 'vuetify/dist/vuetify.min.css';
import colors from 'vuetify/lib/util/colors';

Vue.use(Vuetify);

export default new Vuetify({
    theme: {
        dark : true,
      themes: {
        dark: {
          primary: colors.lightBlue.accent2,//TODO: change
          secondary: colors.red.lighten4, //TODO: change
          accent: colors.indigo.base, //TODO: change
          base: colors.grey.darken4,
          lighten1: colors.grey.darken3,
          lighten2: colors.grey.darken2,
          lighten3: colors.grey.darken1
        },
      },
    },
  })
