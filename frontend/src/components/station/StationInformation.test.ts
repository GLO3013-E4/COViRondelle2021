import Vuetify from "vuetify";
import { mount } from "@vue/test-utils";

import StationInformation from "@/components/station/StationInformation.vue";

const vuetify = new Vuetify();
const wrapper = mount(StationInformation, { vuetify });

describe("When mounting station information", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
