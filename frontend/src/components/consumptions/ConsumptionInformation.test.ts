import Vuetify from "vuetify";
import { mount } from "@vue/test-utils";

import ConsumptionInformation from "@/components/consumptions/ConsumptionInformation.vue";

const vuetify = new Vuetify();
const wrapper = mount(ConsumptionInformation, { vuetify });

describe("When mounting consumption information", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
