import Vuetify from "vuetify";
import { mount } from "@vue/test-utils";

import CycleInformation from "@/components/cycles/CycleInformation.vue";

const vuetify = new Vuetify();
const wrapper = mount(CycleInformation, { vuetify });

describe("When mounting cycle information", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
