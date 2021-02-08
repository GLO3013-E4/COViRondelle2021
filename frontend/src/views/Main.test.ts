import Vuetify from "vuetify";
import { mount } from "@vue/test-utils";

import Main from "@/views/Main.vue";

const vuetify = new Vuetify();
const wrapper = mount(Main, { vuetify });

describe("When mounting Main", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
